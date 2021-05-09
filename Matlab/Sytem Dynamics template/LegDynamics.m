function state_dot = LegDynamics(t,state, parameters)


q=state(1:8);       % Get states, state is a colloum vector lenght 14

dq=state(8:14);     % get states_dot

To=200*(q(1)-q(2))+70*(dq(1)-dq(2));                 % Example PD controller as generalized foces

[W , RHS]= LegODEMatrices(state, To, parameters);    % Extract eq of motion

state_dot=[dq; W\RHS];                               % compute state_dot

end

