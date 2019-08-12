# Note that this script runs in the main context of drake-visualizer,
# where many modules and variables already exist in the global scope.
from director import lcmUtils
from director import applogic
from director import objectmodel as om
from director import visualization as vis
from director.debugVis import DebugData
import numpy as np
from six import iteritems

import drake as lcmdrakemsg

from drake.tools.workspace.drake_visualizer.plugin import scoped_singleton_func

def clip(value, min_val, max_val ):
    if value < min_val:
        return min_val
    elif value > max_val:
        return max_val
    else:
        return value

class ColorMap:
    '''Color map that maps
    [min_val, max_val] -> black, blue, magenta, orange, yellow, white
    linearly. Saturates to black and white for values below min_val or above
    max_val, respectively'''
    def __init__(self, data_range=None):
        if data_range is None:
            self.data_range = [0.0, 10.0]
        else:
            self.data_range = data_range

    def get_color(self, value, range=None):
        if range is None:
            range = self.data_range
        # print('get_color({} - {})'.format(value, range))
        norm_data = self._normalize(value, range)
        return self._do_get_color(norm_data)

    def _do_get_color(self, norm_value):
        raise NotImplementedError("Subclasses need to implement this")

    def update_range(self, min_val, max_val):
        print("Update range to: {} - {}".format(min_val, max_val))
        if self.data_range[0] > min_val:
            self.data_range[0] = min_val
        if self.data_range[1] < max_val:
            self.data_range[1] = max_val

    def _normalize(self, data, (min_val, max_val)):
        """Returns an affine mapped version of the data based on the data range
         provided"""
        if (min_val > max_val):
            raise AttributeError("Bad range: [{}, {}]".format(min_val, max_val))
        assert(max_val >= min_val)
        range = max_val - min_val
        if ( range > 0.00001 ):
            return clip((data - min_val) / (max_val - min_val), 0.0, 1.0)
        else:
            return np.zeros_like(data)

class FlameMap(ColorMap):
    '''Color map that maps
    [min_val, max_val] -> black, blue, magenta, orange, yellow, white
    linearly. Saturates to black and white for values below min_val or above
    max_val, respectively'''
    def _do_get_color(self, norm_data):
        color = [0, 0, 0]
        color[0] = clip(((norm_data - 0.25) * 4.0), 0.0, 1.0)
        color[1] = clip(((norm_data - 0.5) * 4.0), 0.0, 1.0)
        if norm_data < 0.25:
            color[2] = clip(norm_data * 4.0, 0.0, 1.0)
        elif norm_data > 0.75:
            color[2] = clip((norm_data - 0.75) * 4.0, 0.0, 1.0)
        else:
            color[2] = clip(1.0 - (norm_data - 0.25) * 4.0, 0.0, 1.0)
        return color

class IntensityMap(ColorMap):
    def _do_get_color(selfself, norm_data):
        return np.array((0.0, 1.0, 0.0), dtype=np.float) * norm_data

class TwoToneMap(ColorMap):
    def __init__(self):
        ColorMap.__init__(self)
        self.min_color = np.array((240, 1, 1.0))
        self.max_color = np.array((320.0, 1, 1.0))
        self.delta = self.max_color - self.min_color

    def _do_get_color(self, norm_data):
        hsv = self.min_color + self.delta * norm_data
        return self.hsvToRgb(hsv)

    def hsvToRgb(self, hsv):
        '''Convert hue, saturation and lightness to r, g, b values.
        Hue in [0, 360], s in [0, 1], l in [0, 1].
        If dtype is int, then values are in the range [0, 255]
        otherwise in the range [0,1]'''
        h, s, v = hsv
        r = g = b = 0.0
        c = s * v
        h /= 60.0
        x = c * (1 - abs( ( h % 2 ) - 1 ) )

        if ( h >= 0 and h < 1 ):
            r = c
            g = x
        elif ( h >= 1 and h < 2 ):
            r = x
            g = c
        elif ( h >= 2 and h < 3 ):
            g = c
            b = x
        elif ( h >= 3 and h < 4 ):
            g = x
            b = c
        elif ( h >= 4 and h < 5 ):
            r = x
            b = c
        else:
            r = c
            b = x
        m = v - c
        r += m
        g += m
        b += m
        return r, g, b

class ContactVisualizer(object):
    def __init__(self):
        self._folder_name = 'Contact Results'
        self._name = "Contact Visualizer"
        self._enabled = False
        self._sub = None
        self.history_length = 10  # number of frames
        # self.color_map = TwoToneMap()
        self.color_map = IntensityMap()
        # self.color_map = FlameMap()

        self.set_enabled(True)
        # Basis for updating the color map
        self.history = []

    def add_subscriber(self):
        if self._sub is not None:
            return

        self._sub = lcmUtils.addSubscriber(
            'CONTACT_RESULTS',
            messageClass=lcmdrakemsg.lcmt_contact_results_for_viz,
            callback=self.handle_message)
        print self._name + " subscriber added."

    def remove_subscriber(self):
        if self._sub is None:
            return

        lcmUtils.removeSubscriber(self._sub)
        self._sub = None
        om.removeFromObjectModel(om.findObjectByName(self._folder_name))
        print self._name + " subscriber removed."

    def is_enabled(self):
        return self._enabled

    def set_enabled(self, enable):
        self._enabled = enable
        if enable:
            self.add_subscriber()
        else:
            self.remove_subscriber()

    def handle_message(self, msg):
        # Limits the rate of message handling, since redrawing is done in the
        # message handler.
        self._sub.setSpeedLimit(30)

        # Removes the folder completely.
        om.removeFromObjectModel(om.findObjectByName(self._folder_name))

        # Recreates folder.
        folder = om.getOrCreateContainer(self._folder_name)

        biggest = -1.0
        smallest = np.inf

        # print("Message")
        # A map from pair of body names to a list of contact forces
        collision_pair_to_forces = {}
        for contact in msg.contact_info:
            point = np.array([contact.contact_point[0],
                              contact.contact_point[1],
                              contact.contact_point[2]])
            force = np.array([contact.contact_force[0],
                              contact.contact_force[1],
                              contact.contact_force[2]])
            mag = np.linalg.norm(force)
            if mag > biggest: biggest = mag
            if mag < smallest: smallest = mag
            color = self.color_map.get_color(mag)
            # print "  force: {}, range: [{:.5f}, {:.5f}]".format(mag, smallest, biggest)
            # intensity = (mag - self.domain_min) / self.domain_size
            if mag > 1e-4:
                mag = 0.3 / mag

            key1 = (str(contact.body1_name), str(contact.body2_name))
            key2 = (str(contact.body2_name), str(contact.body1_name))

            if key1 in collision_pair_to_forces:
                collision_pair_to_forces[key1].append(
                    (point, point + mag * force, color))
            elif key2 in collision_pair_to_forces:
                collision_pair_to_forces[key2].append(
                    (point, point + mag * force, color))
            else:
                collision_pair_to_forces[key1] = \
                    [(point, point + mag * force, color)]

        for key, list_of_forces in iteritems(collision_pair_to_forces):
            d = DebugData()
            for force_pair in list_of_forces:
                d.addArrow(start=force_pair[0],
                           end=force_pair[1],
                           tubeRadius=0.005,
                           headRadius=0.01)

            vis.showPolyData(d.getPolyData(), str(key), parent=folder,
                             color=force_pair[2]) #[0.2 * intensity, 0.8 * intensity,
                                    #0.2 * intensity])
        if len(self.history) > self.history_length:
            self.history.pop(0)
        self.history.append((smallest, biggest))
        for s, b in self.history:
            if s < smallest: smallest = s
            if b > biggest: biggest = b
        # self.color_map.update_range(smallest, biggest)
        if (smallest < biggest):
            self.color_map.data_range = [smallest, biggest]
        vis.updateText('Color Range: [{:.3f}, {:.3f}] N'.format(
            self.color_map.data_range[0], self.color_map.data_range[1]),
                       'text')


@scoped_singleton_func
def init_visualizer():
    # Create a visualizer instance.
    my_visualizer = ContactVisualizer()
    # Adds to the "Tools" menu.
    applogic.MenuActionToggleHelper(
        'Tools', my_visualizer._name,
        my_visualizer.is_enabled, my_visualizer.set_enabled)
    return my_visualizer


# Activate the plugin if this script is run directly; store the results to keep
# the plugin objects in scope.
if __name__ == "__main__":
    contact_viz = init_visualizer()
