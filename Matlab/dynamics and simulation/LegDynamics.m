function state_dot = BBDynamics(t,state, parameters)


q=state(1:4);

dq=state(5:8);

To=200*(q(1)-q(2))+70*(dq(1)-dq(2));

[W , RHS]= BallAndBeamODEMatrices(state, To, parameters);

state_dot=[dq; W\RHS];

end

