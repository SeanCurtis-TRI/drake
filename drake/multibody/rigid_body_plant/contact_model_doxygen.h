/** @defgroup drake_contacts   Compliant Point Contacts in Drake

 Drake is concerned with the simulation of _physical_ phenomena, including
 contact between simulated objects.
 Drake approximates real-world physical contact phenomena with a combination
 of geometric techniques and response models. Here we discuss the
 parameterization and idiosyncracies of a particular contact response model,
 based on point contact with compliance and dissipation, and a Stribeck friction
 model approximating Coulomb stiction and sliding friction effects.

 This document gives an overview of the _current_, very limited state of the
 implementation of compliant contact in Drake with particular emphasis on how to
 account for its particular
 quirks in a well-principled manner. What works in one simulation scenario, may
 not work equally well in another scenario. This discussion will encompass:

 - @ref contact_geometry "properties of the geometric contact techniques",
 - @ref contact_model "details of the contact response model", and
 - @ref per_object_contact "per-object contact materials"
 - @ref contact_engineering "techniques for teasing out desirable behavior".

 @section contact_spec  Definition of contact

 Before getting into the details of how contacts are detected and responses are
 modeled, it is worthwhile to define what a contact means to Drake.

 First, contact _conceptually_ occurs between two _surfaces_, one on each of
 two independently moving _bodies_. The contact produces _forces_ on those two
 surfaces, which can then affect the motion of the bodies. In practice, surfaces
 are represented by one or more drake::multibody::collision::Element objects --
 geometric shapes rigidly affixed to a body, whose surfaces can engage in
 contact.

 This document discusses a _compliant_ contact model. In compliant models, the
 bodies are considered deformable. The
 collision geometry represents an object in its undeformed state. As two objects
 collide, the contact forces cause them to deform. Compliant models compute
 the forces that would cause the deformation. The deformed geometry is _not_
 modeled explicitly. Instead, the contact forces are computed based on the
 degree of penetration of the non-deforming collision geometry; greater
 penetration implies larger contact forces.
 One can think of largely rigid objects which have slightly deformable surfaces.
 For this model to be useful in practice, the deformations should be small
 relative to the whole body, so that we can (a) use simple models for the
 relationship between deformation and forces, and (b) neglect the change in mass
 properties induced by the deformation. As such, all discussion of collision
 geometry/elements refers to this _undeformed_ geometry.

 Contacts are defined in terms of these collision Element instances and _not_
 RigidBody instances. For Drake's purposes, a "contact":

 - describes a relationship between two drake::multibody::collision::Element
   instances, denoted elements `A` and `B`,
 - only exists if the Element instances overlap,
 - quantifies the degree that the two Element instances are overlapping,
 - is characterized by a single contact point and a normal direction that are
   used to define a _contact frame_ `C`, and
 - leads to the generation of two equal-but-opposite forces acting on the
   RigidBody instances to which the corresponding Elements belong.

 Handling physical contact is decomposed into (1) the detection and
 quantification of overlap (penetration) through the use of geometric techniques
 applied to collision geometry, and (2) the generation of the resultant
 forces based on models of material and surface properties resulting in
 deformation and frictional forces.

 Next topic: @ref contact_geometry
*/

/** @defgroup contact_geometry Detecting Contact
 @ingroup drake_contacts

 Given two posed geometric shapes in a common frame, the collision detection
 system is responsible for determining if those shapes are penetrating and
 characterizing that penetration. We won't go into the details of the how and
 why these techniques work the way they do, but, instead, focus on _what_ the
 properties of the results of the _current implementation_ are.  It is worth
 noting that some of these properties are considered _problems_ yet to be
 resolved and should not necessarily be considered desirable.

 -# Between any two collision Elements, only a _single_ contact will be
 reported.
 -# Contacts are reported as contact at a _point_. (This is a very reasonable
 assumption for smooth convex shapes such as spheres and ellipsoids where
 relative motion must inevitably lead to initial contact at a single point.)
 -# Surface-to-surface contacts (such as a block sitting on a plane) are
 unfortunately still limited to a single contact point, typically located at
 the point of deepest penetration. (That point will necessarily change from step
 to step in an essentially non-physical manner that is likely to cause
 difficulties.)
 -# A contact _normal_ is determined that approximates the mutual normal of
 the contacting surfaces at the contact point.

 Next topic: @ref contact_model
 */

/** @defgroup contact_model Computing Contact Forces
 @ingroup drake_contacts

 Consider @ref contact_spec "the definition of a contact" with
 interpenetrating collision elements `A` and `B`. The single, computed contact
 point serves as the origin of a contact frame `C`, so is designated `Co`; we'll
 shorten that to just `C` when it is clear we mean the point rather than the
 frame. We define the normal as pointing outward from the deformed surface of
 `B` towards the deformed interior of `A`. The `C` frame's z-axis is aligned
 along this normal (with arbitrary x- and y-axes). We are also interested in the
 points of bodies `A` and `B` that are coincident with `Co`, which we call
 `Ac` and `Bc`, respectively. Because the two
 forces are equal and opposite, we limit our discussion to the force `f` acting
 on `A` at `Ac` (such that `-f` acts on `B` at `Bc`).

 @image html simple_contact.png "Figure 1: Illustration of contact between two spheres."

 The computation of the contact force is most naturally discussed in the
 contact frame `C` (shown in Figure 1).

 The contact force, `f`,  can be decomposed into two components: normal, `fₙ`,
 and tangential, `fₜ` such that `f=fₙ+fₜ`. The normal force lies in the
 direction of the contact frame's z-axis.  The tangential
 component lies in the contact frame's x-y plane.  In Drake's compliant contact
 model, although these components are orthogonal, they are _not_ independent;
 the tangential force is a function of the normal force.

 The model described here is adapted from Simbody's
 Hertz/Hunt & Crossley/Stribeck model described in [Sherman 2011]. We will
 summarize the elements of this model below.

 - [Sherman 2011] M. Sherman, et al. Procedia IUTAM 2:241-261 (2011), Section 5.
   http://dx.doi.org/10.1016/j.piutam.2011.04.023

 @section normal_force Hunt-Crossley Normal Force

 Drake uses the Hunt-Crossley model [Hunt 1975] for computing a normal force
 `fₙ` that accounts for both stiffness and dissipation effects. This is a
 continuous model based on Hertz elastic contact theory, which correctly
 reproduces the empirically observed velocity dependence of the coefficient of
 restitution, where `e=(1-d⋅v)` for (small) impact velocity `v` and a material
 property `d` with units of 1/velocity. In theory, at least, `d` can be
 measured right off the coefficient of restitution-vs.-impact velocity curves:
 it is the negated slope at low impact velocities.

 Given a collision between two spheres, or a sphere and a plane, we can generate
 a contact force from this equation `fₙ = kxᵐ(1 + ³/₂⋅d⋅ẋ)` where `k` is a
 stiffness constant incorporating material properties and geometry
 (to be defined below), `x` is penetration depth, and `ẋ` is penetration rate
 (positive for increasing penetration and negative during rebound). Exponent `m`
 depends on the surface geometry and captures not only the functional form of
 the contact pressure (normal stress) with penetration, but also the change in
 contact patch area with penetration.
 For the contact between two surfaces with radii of curvature R₁ and R₂, the
 Hertz contact (normal) force can be written in terms of an effective radius of
 curvature R defined as `1/R = 1/R₁ + 1/R₂`. Similarly, an effective Young's
 modulus E is defined from the modulii E₁ and E₂ and the Poisson's ratios of
 each surface material as `1/E = (1-ν₁²)/E₁ + (1-ν₂²)/E₂`.
 With these definitions, Hertz model predicts:
 - Contact between two spheres (this includes the plane by taking the limit
   to infinity on one of the radii):
   `m = 3/2`, `k = ⁴/₃ E √R`, or `fₙ = ⁴/₃ E R² (x/R)ᵐ`.
   The contact radius is given by `a = Sqrt(R x)`.
 - Two crossed cylinders of equal radii R: Same as for sphere of radius R and a
   plane.
 - Vertical cylinder of radius R and a plane:
   `m = 1`, `k = 2 E R x`, or `fₙ = 2 E R² (x/R)`.
 - Two cylinders of radii R₁ and R₂, length L aknd with parallel axes:
   `m = 1`, `k = π/4 E L`, or `fₙ = π/4 E R² (R/L) (x/R)`.

 These few examples with analytical solution lead us to generalize to an
 expression of the contact normal force of the from:

   `fₙ = C⋅E⋅L²⋅(x/R)ᵐ⋅(1 + d⋅ẋ)`,

 where L is a reference length that represents an _effective_ radius of
 curvature, E is the effective Young's modulus defined above, and C is a
 dimensionless scaling constant.

 Drake's _current_ compliant model implementation makes aggressive simplifying
 assumptions. Although, C and L are related to the contacting geometries, the
 current model simply assumes `C = 1` and `L = 1 m` (leading to issues
 documented below, see @ref contact_engineering for details).

 // TODO(SeanCurtis-TRI): We'll be changing these constants to some *other*
 // constants that admit the possibility of meaningful physical values for E
 // (on the order of giga-pascals). It's still coarse approximations and won't
 // change the quality of the output, it just scales the algebra.

 Please note, `d` is not a _damping_ coefficient (which would have units of
 force/velocity), but is a _dissipation_ factor, with units of 1/velocity,
 modeling power loss as a rate-dependent fraction of the conservative
 deformation-dependent force. For steel, bronze or ivory, [Hunt 1975] reports
 values of d between 0.08-0.32 sec/m.

 // TODO(amcastro-tri): See if possible to estimate d in terms of a
 // dimensionless damping ratio (with =1 critically damped, <1 underdamped,
 // >1 overdamped).
 // Therefore users would only need to provide E (material tables) and a
 // dimensionless damping ratio, ad-hoc estimation, but at least in the range
 // 0 to O(1).

 By definition `fₙ` should always be positive, so that the contact force is
 a repulsive force. Mathematically, for arbitrary x and ẋ, it is possible for
 `fₙ` to become negative, creating an attractive or "sucking" force. This case
 will be achieved if `ẋ < -1 / d`. To prevent sucking forces, the normal
 component is clamped to zero. In this regime, it is still possible for there to
 be a repulsive force for bodies that are drawing apart (`ẋ < 0`), as long as
 the relative velocities are small. This approximately models recovery of energy
 in the deformed material with observed hysteresis effects as described in
 [Hunt 1975]. However, a surface can't return to its undeformed state
 arbitrarily quickly. If the bodies are pulled apart *faster* than the surface
 can recover, the bodies will separate before the potential energy of
 deformation can be converted to kinetic energy, resulting in energy loss (to
 heat, vibration, or other unmodeled effects).

 - [Hunt 1975] K. H. Hunt and F. R. E. Crossley, "Coefficient of Restitution
   Interpreted as Damping in Vibroimpact," ASME Journal of Applied Mechanics,
   pp. 440-445, June 1975. http://dx.doi.org/10.1115/1.3423596

 The units for spring-like stiffness k in the Hunt-Crossley model are N/m
 (force/length). The units for dissipation d are sec/m (1/velocity). Therefore,
 the units in the equation `fₙ = kx(1 + dẋ)` are, leading to a result with units
 of force, or N in SI.

 More generally, we can write

   `fₙ = p(q)·A(q)`

 where `A(q)` is the contact patch area, `p(q)` is the average contact pressure
 on that patch and q is the contact configuration, the details of which depend
 on how the contact is characterized (for instance, it could be the penetration
 depth, and the approaching speed). Notice there is no approximation in
 `fₙ = p(q)·A(q)`, this is always true since this equation _is_ the definition
 of the average contact pressure `p(q)`.
 Using dimensional analysis we write this expression in terms of a dimensionless
 pressure `p*(q)` and a dimensionless area `A*(q)` as:

   `fₙ = E·L²·p*(q)·A*(q)`

 where L is an appropriate reference length for the particular contact
 configuration.
 Lets attempt to recast the analytical results mentioned above in these terms:
 - Contact between two spheres (this includes the plane by taing the limit
   to infinity on one of the radius):
   A(q) = π a² = π R x,
   p(q) = 4/(3π) E (x/R)½
 - Two cylinders of radii R₁ and R₂, length L and with parallel axes:
   A(q) = π a L = L (R x)½,
   p(q) = π/4 E (x/R)½
 - Vertical cylinder of radius R and a plane:
   A(q) = π R²,
   p(q) = 2/π E (x/R)

 Interestingly, even when the functional form for the contact patch area for
 two very distintct cases of sphere-on-sphere and cylinder-on-cylinder are
 different (linear vs ½ power with penetration depth), the average pressure
 scales with the same power.
 Of course this result could not be generalized and for a case with planar
 surfaces like a vertical cylinder on a plane, the average pressure has a
 different functional form with penetration depth.

 In Drake's current contact model the contact configuration is described by
 `q = [x; ẋ]` where x is the simply the penetration depth at a single point and
 ẋ is the rate of change of this depth. Similar to the case of a vertical
 cylinder contacting on a planar surface, it assumes that the contact area is
 independent of the depth and that pressure is proportional to the penetration
 depth leading to the simple model `fₙ = C⋅E⋅L²⋅(x/R)ᵐ⋅(1 + d⋅ẋ)`, taking
 `L = 1 m` for the reference length and `C = 1`. These simplifying
 assumptions lead to artifacts. See @ref contact_engineering for details.

 Therefore, Drake's current model properly scales contact force with the
 materials's stiffness, namely the Young modulus, however it fails to include
 the more complex functional forms on penetraion depth introduced by the
 particulars of the contact geometries. Future implementations will replace the
 current implementation with more physically plausible models.

 @section tangent_force Stribeck Friction Tangential Force

 Static friction (or stiction) arises due to surface characteristics at the
 microscopic level (e.g., mechanical interference of surface imperfections,
 electrostatic, and/or Van der Waals forces). Two objects in static contact
 need to have a force `fₚ` applied parallel to the surface of contact sufficient
 to _break_ stiction. Once the objects are moving, dynamic (kinetic) friction
 takes over. It is possible to accelerate one body sliding
 across another body with a force that would have been too small to break
 stiction. In essence, stiction will create a contrary force canceling out
 any force too small to break stiction (see Figure 2).

 <!-- This is illustrated much better in the formatted doxygen image, but in
 case you are too lazy to look there:

     Pushing Force vs Tangent Force

      |     stiction
   Fₛ |   |
      |   |      dynamic friction
      |   |______________________
  fₜ  |   |
      |   |
      |   |
      |   |
    0 |___|________________________
          0                      Fₚ
                      fₚ
      Figure 2: Idealized Stiction/Sliding Friction Model
 -->
 @image html ideal_stiction.png "Figure 2: Idealized Stiction/Sliding Friction Model"

 In _idealized_ stiction, tangent force `fₜ` is equal and opposite
 to the pushing force `fₚ` up to the point where that force is sufficient to
 break stiction (the red dot in Figure 2). At that point, the tangent force
 immediately becomes a constant force based on the _dynamic_ coefficient of
 friction. There are obvious discontinuities in this function which do not occur
 in reality, but this can be a useful approximation and can be implemented in
 this form using constraints that can be enabled and disabled discontinuously.
 However, here we are looking for a continuous model that can produce reasonable
 behavior using only unconstrained differential equations. With this model we
 can also capture the empirically-observed Stribeck effect where the friction
 force declines with increasing slip velocity until it reaches the dynamic
 friction level.

 <!-- This is illustrated much better in the formatted doxygen image, but in
 case you are too lazy to look there:

   Stribeck function: μ vs. vₛ

      |
      |
   μs |     **
      |    *  *
      |    *   *
   μd |   *      **********
      |   *
      |   *
      |   *
      |  *
      |*____________________
      0     1     2     3
          multiple of vₛ

   Figure 3: Stribeck function for stiction.
 -->
 @image html stribeck.png "Figure 3: Stribeck function for stiction"

 <!-- TODO(SeanCurtis-TRI,sherm1) Consider using "static" and "kinetic"
 coefficients of friction so we can write μₛ and μₖ in Unicode ("d" isn't
 available as a subscript). This isn't simply a change in this file; the
 code should also reflect this nomenclature change. -->

 The Stribeck model is a variation of Coulomb friction, where the frictional
 (aka _tangential_) force is proportional to the normal force as:

 `fₜ = μ⋅fₙ`

 In the Stribeck model, the coefficient of friction, μ, is replaced with a
 slip speed-dependent function:

 `fₜ = μ(s)⋅fₙ`,

 where `s` is a unitless multiple of a _new_ parameter: _slip tolerance_ (`vₛ`).
 Rather than modeling _perfect_ stiction, it makes use of an _allowable_ amount
 of relative motion to approximate stiction.  When we refer to
 "relative motion", we refer specifically to the relative motion of the two
 points `Ac` and `Bc` on the corresponding bodies that are coincident in space
 with the contact point `C`.

 The function, as illustrated in Figure 3, is a function of the unitless
 _multiple_ of `vₛ`. The domain is divided into three intervals:

    - `s ∈ [0, 1)`: the coefficient of friction rises smoothly from zero to the
    static coefficient of friction, μs.
    - `s ∈ [1, 3)`: The coefficient of friction falls smoothly from
    μs to the dynamic (kinetic) coefficient of friction, μd.
    - `s ∈ [3, ∞)`: Coefficient of friction is held constant at μd.

 Other than the residual "creep" velocity limited by `vₛ`, which can be
 arbitrarily small (in theory; see next section for practical considerations),
 this model produces a reasonably good approximation of
 Coulomb friction. Its primary drawback is that the model is numerically
 very stiff in the stiction region, which requires either small step sizes
 with an explicit integrator, or use of a more-stable implicit integrator.

 Next topic: @ref contact_engineering
*/

/** @defgroup per_object_contact Per-object contact material
 @ingroup drake_contacts

 Drake supports defining compliant contact material properties on a per
 collision geometry basis. It has several mechanisms in place to facilitate
 working with per-object contact materials:

 - Universal default values (all objects default to the universal values if none
   have been explicitly specified.
 - Parsing per-collision element properties from URDF and SDF files using
   extended tags (formatted identically for both source files types).
 - Runtime access to set the global default values and per-element values.

 @section Material parameters and evaluating contact

 The per-object material properties resemble those of the contact model,
 consisting of:

 - stiffness (k) with units of stress/strain,
 - dissipation (d) with units of 1/velocity, and
 - static and dynamic friction (unitless μ_s and μ_d, respectively).

 The parameters outlined in @ref contact_model are derived from the parameter
 values for the two colliding bodies. Consider two colliding bodies I and J.
 The contact values k, d, μ_s, and μ_d used to compute the contact force
 are defined in the following way:

 - sᵢ ∈ [0, 1] is the "squish" factor of body I. It represents the amount of
   total deformation is experienced by body I. Consider contact between a steel
   body and foam ball; the foam ball would experience the entire deformation and
   the squish factors for the foam ball and steel plate would be 1 and 0,
   respectively. The squish value is defined as sᵢ = kⱼ / (kᵢ + kⱼ), with
   sⱼ = 1 - sᵢ.
 - k = sᵢkᵢ = sⱼkⱼ. The stiffness of the _contact_ will generally not be the
   stiffness of either constituent material (unless one were infinite). If
   kᵢ = kⱼ, then k would be kᵢ/2.
 - d = sᵢdᵢ + s₂d₂. Again, the dissipation of the contact is simply a linear
   interpolation of the two bodies' dissipation values.
 - μ_s (and μ_d) are defined as 2μᵢμⱼ / (μᵢ + μⱼ).

 Finally, the contact point is also defined with respect to the "squish"
 factors. For penetrating bodies I and J, there is a point on the surface of I
 that _most_ deeply penetrates into J (and vice versa). We will call those
 points p_FIc and p_FJc (measured and expressed in some common frame F). The
 contact point, defined in the same frame, is p_FC = p_FIc * sⱼ + P_FJc * sᵢ.
 We draw _particular_ attention to the fact that the point on I's surface is
 weighted by J's squish factor and vice versa. That is because, if body I
 experiences all of the deformation, it will be deformed all the way to the
 point of deepest penetration _in_ I, which was the definition of p_FJc.

 @section Global default values

 Drake contains hard-coded material parameter values
 (see compliant_parameters.h). If no specific values are provided for
 a collision element, it will _shadow_ these global default values. This
 relationship is _dynamic_. In other words, the default _values_ are not copied
 into an element at instantiation time, just a _relationship_. A collision
 element configured to use default parameter values will report different values
 as the default values are changed. The implication is the order of operations
 between instantiating collision elements which use default values and setting
 default values is irrelevant.

 To set the default values, use CompliantMaterialParameters::SetDefaultValues().

 __A word of warning__

 It might be tempting to write code akin to this pseudo-code:

 ```C++
 CompliantMaterialParameters my_defaults;
 my_defaults.set_stiffness(10);
 CompliantMaterialParameters::SetDefaultValues(my_defaults);
 ParseUrdf("my_robot.urdf");
 my_defaults.set_stiffness(15);
 CompliantMaterialParameters::SetDefaultValues(my_defaults);
 ParseUrdf("other_robot.urdf");
 ```

 Assume that the collision elements in both `my_robot.urdf` and
 `other_robot.urdf` have no specified contact parameters; they use the default
 values. At first glance, one might be inclined to believe that the first
 robot's collision elements have a stiffness value of 10 and the second robot
 has a stiffness value of 15. This is _not_ the case. Both robots use the
 single global default value and so both have stiffness values of 15.

 @section Specifying contact parameter values in URDF/SDF.

 We are exploiting the fact that URDF and SDF are XML files and choose to
 naively extend the specification to include a custom tag. Although there are
 numerous differences between the two formats, there is remarkable similarity
 in declaring collision geometries. For simplicity's sake, we expect identically
 formatted contact material format in both formats that look something like
 this:

 ```xml
 ...
 <collision ...>
   <geometry ...>
   </geometry>

   <drake_compliance>
     <stiffness>##</stiffness>
     <dissipation>##</dissipation>
     <static_friction>##</static_friction>
     <dynamic_friction>##</dynamic_friction>
   </drake_compliance>

 </collision>
 ...
 ```

 Differences between URDF and SDF are dismissed with ellipses. What is
 significant is that the `<drake_compliance>` tag should be introduced as a
 child of the `<collision>` tag (common to both formats) and should be
 formatted as shown.

 The following rules are applied for parsing:

 - If no `<drake_compliance>` tag is found, the element uses the global default
   parameters.
 - Not all properties are required; explicitly specified properties will be
   applied to the corresponding element and omitted properties will map to the
   default values.
 - Friction values must be defined as a pair, or not at all. When defined as a
   pair, the `static_friction` value must be greater than or equal to the
   `dynamic_friction` value. Failure to meet these requirements will cause a
   runtime exception.
 */

/** @defgroup contact_engineering Working with Contacts in Drake
 @ingroup drake_contacts

 The behavior of a simulation with contact will depend on three factors:

 - the choice of integrator,
 - contact parameters,
 - nature of collision geometry.

 The three factors are interdependent; specific choices for one factor may
 require supporting changes in the other factors.

 Issues:
 - **Picking a good value for `vₛ`**

   In selecting a value for `vₛ`, you must ask yourself the question, "When
   two objects are ostensibly in stiction, how much slip am I willing to allow?"
   There are two opposing design issues in picking a value for `vₛ`.  On the one
   hand, small values of `vₛ` make the problem numerically stiff during
   stiction. Stable integration then requires either _very_ small step sizes
   when using an explicit integrator, or use of an implicit integrator. Implicit
   integration is under development but not yet available in Drake, so a small
   `vₛ` will require small steps, or high accuracy for the error-controlled RK3.
   On the other hand, it
   should be picked to be appropriate for the scale of the problem. For example,
   a car simulation could allow a "large" value for `vₛ` of 1 cm/s (1e-2 m/s),
   but reasonable stiction for grasping a 10 cm box might require limiting
   residual slip to a mm/s or less, 1e-3 or 1e-4 m/s. Ultimately,
   picking the largest viable value will allow your simulation to run faster.

 - **Picking values for the other contact parameters**

   The contact model provides five parameters:
     - Stiffness `k` in units of stress/strain,
     - dissipation `d` in s/m (1/velocity),
     - static coefficient of friction `μs`, unitless,
     - dynamic (kinetic) coefficient of friction `μd`, unitless,
     - and stiction slip speed tolerance `vₛ` in m/s.

   In a compliant model, deformation (which appears as
   penetration of the undeformed geometry) is part of the stable equilibrium
   state. Imagine a box sitting on a plane.
   The stable penetration depth will, in principle, be equal to the box's weight
   divided by the stiffness. Appropriate stiffness for a 1-kg box is not the
   same as for a 1000-kg car. (In fact, with a small stiffness, the car will
   pass right through the ground while attempting to find the equilibrium
   distance.) Stiffness is the most important parameter for capturing the
   relationship between object in equilibrium. The dissipation `d` is
   significant primarily for impacts, where there are rapid changes in
   deformation.

   Simulation performance using an explicit integrator is likely to be most
   affected by `k` and `vₛ`. In cases where more penetration is acceptable, you
   can soften `k` and get better performance in exchange for less-realistic
   deformation. The total contact force at equilibrium is not very sensitive
   to `k` since the penetration will be adjusted as necessary to achieve
   force balance. For stiction behavior, increasing the coefficients of friction
   to unrealistic levels seems, counterintuitively, to degrade the results. The
   previous note discusses the importance of `vₛ`.

 - ** Stiffness and model limitations **

   As indicated earlier, the _current_ contact model generates a contact force
   by assuming `A(x) = 1 m²` and `ε(x) = x / 1 m`. The implication is that the
   forces acting on bodies in contact are not purely a function of stiffness and
   degree of penetration but also of the _number_ of contact points reported,
   since each contact point ostensibly represents some fraction of the contact
   area. Selecting a stiffness value from a reference table may not produce
   the desired result. The simulated material may appear significantly softer.
   As the number of contact points increase, we sample the contact manifold at a
   higher density and get a truer representation of the area of contact and,
   therefore, a more meaningful contact force. This is particularly significant
   when considering the next point.

   Until this model changes, it may be necessary to tune the stiffness to a
   higher value than one that is reported as being physically accurate. When
   the underlying model is changed, this issue will be removed.

 - **Surface-on-surface contacts**

   Remember that the contact detection computation produces a single point to
   represent contact between two collision elements. If the contact is a
   surface instead of a point (such as one box lying on another), the contact
   point will *not* be temporally coherent. This will lead to instability
   artifacts which can only be addressed through smaller time steps.

   An alternative is to represent the body's contact differently. For some
   shapes (e.g., boxes), we can introduce two sets of collision elements:
   discrete "points" at the corners, and a box capturing the volume (see
   `block_for_pick_and_place.urdf` as an example). With this strategy, the
   contact "points" are actually small-radius spheres. The volume-capturing
   box should actually be inset from those spheres such that when the box is
   lying on a plane (such that the logical contact manifold would be a face),
   only the contact points make contact, providing reliable points of contact.
   However, for arbitrary configurations contact with the box will provide
   more general contact.

 - **Choice of integrator**

   Empirical evidence suggests that any integrator _except_
   ExplicitEulerIntegrator can work with this contact model. Generally, the
   RungeKutta2Integrator and SemiExplicitEulerIntegrator require similar time
   steps to produce equivalent behavior. Generally, for a `vₛ` value of
   1e-2 m/s, a timestep on the order of 1e-4 is required for both of these.
   The error-controlled RungeKutta3Integrator will choose very small steps and
   accuracy must be set tight enough to ensure stability. An implicit integrator
   is currently in development and should perform much better on
   stiction-dominated problems such as manipulator grasping.
 */
