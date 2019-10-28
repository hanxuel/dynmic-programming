function [ J_opt, u_opt_ind ] = ValueIteration( P, G )
%VALUEITERATION Value iteration
%   Solve a stochastic shortest path problem by Value Iteration.
%
%   [J_opt, u_opt_ind] = ValueIteration(P, G) computes the optimal cost and
%   the optimal control input for each state of the state space.
%
%   Input arguments:
%
%       P:
%           A (K x K x L)-matrix containing the transition probabilities
%           between all states in the state space for all control inputs.
%           The entry P(i, j, l) represents the transition probability
%           from state i to state j if control input l is applied.
%
%       G:
%           A (K x L)-matrix containing the stage costs of all states in
%           the state space for all control inputs. The entry G(i, l)
%           represents the cost if we are in state i and apply control
%           input l.
%
%   Output arguments:
%
%       J_opt:
%       	A (K x 1)-matrix containing the optimal cost-to-go for each
%       	element of the state space.
%
%       u_opt_ind:
%       	A (K x 1)-matrix containing the index of the optimal control
%       	input for each element of the state space.

% put your code here
num_state=size(G,1);
J_opt=zeros(1,num_state);
J_temp=zeros(1,num_state);
u_opt_ind=zeros(1,num_state);
for i=1:num_state
    viablectr = [];
    for k = 1:size(G,2)
        if (sum(P(i,:,k)) > 0)
            viablectr=[viablectr,k];
        end
    end
    [J_temp(i), ctr_opt] = min( G(i,viablectr) + J_opt*squeeze(P(i,:,viablectr)) );
    u_opt_ind(i) = viablectr(ctr_opt);
end  
while any(abs(J_opt-J_temp)>0.00001)
    J_opt=J_temp;
    for i=1:num_state
        viablectr = [];
        for k = 1:size(G,2)
        	if (sum(P(i,:,k)) > 0)
                viablectr=[viablectr,k];
            end
        end
        [J_temp(i), ctr_opt] = min( G(i,viablectr) + J_opt*squeeze(P(i,:,viablectr)) );
        u_opt_ind(i) = viablectr(ctr_opt);
    end    
end
J_opt=J_opt';
u_opt_ind=u_opt_ind';
