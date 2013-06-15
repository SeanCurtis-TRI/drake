function testGurobiQP


compareSolvers(eye(2),zeros(2,1));




end


function compareSolvers(Q,f,Aeq,beq,Ain,bin,lb,ub,active)

n = length(f);
if nargin<3, Aeq=zeros(0,n); end
if nargin<4, beq=[]; end
if nargin<5, Ain=zeros(0,n); end
if nargin<6, bin=[]; end
if nargin<7, lb=[]; end
if nargin<8, ub=[]; end
if nargin<9, active=[]; end

Ain_b = [Ain; -eye(length(lb)); eye(length(ub))];
bin_b = [bin; -lb; ub];

% todo:  call fastQP here?
[x_fastqp,info_fastqp,active_fastqp] = fastQPmex(Q,f,Aeq,beq,Ain_b,bin_b,active);
[x_gurobimex,info_gurobimex,active_gurobimex] = gurobiQPmex(Q,f,Aeq,beq,Ain,bin,lb,ub,active);
[x_gurobi,info_gurobi,active_gurobi] = mygurobi(Q,f,Aeq,beq,Ain,bin,lb,ub,active);

valuecheck(x_fastqp,x_gurobi);
valuecheck(x_gurobimex,x_gurobi);

end


function [x,info,active] = mygurobi(Q,f,Aeq,beq,Ain,bin,lb,ub,active)

params.outputflag = 0; % not verbose
params.method = 2; % -1=automatic, 0=primal simplex, 1=dual simplex, 2=barrier
params.presolve = 0;
if params.method == 2
  params.bariterlimit = 20; % iteration limit
  params.barhomogeneous = 0; % 0 off, 1 on
  params.barconvtol = 5e-4;
end

if iscell(Q), error('need to implement this case'); end
if isvector(Q), Q = diag(Q); end
model.Q = sparse(Q);
model.obj = f;
model.A = sparse([Aeq;Ain]);
model.rhs = [beq,bin];
model.sense = [repmat('=',length(beq),1); repmat('<',length(bin),1)];
if ~isempty(lb), model.lb = lb; end
if ~isempty(ub), model.ub = ub; end

result = gurobi(model,params);

info = result.status;
x = result.x;
%active = result.slacks;

end
