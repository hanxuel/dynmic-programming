function [ J_opt, u_opt_ind ] = LinearProgramming( P, G )
%LINEARPROGRAMMING Value iteration
%   Solve a stochastic shortest path problem by Linear Programming.
%
%   [J_opt, u_opt_ind] = LinearProgramming(P, G) computes the optimal cost
%   and the optimal control input for each state of the state space.
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
num_ctr=size(P,3);
u_opt_ind = zeros(num_state, 1);
f=-ones(num_state,1);
A=[];
b=[];
for i=1:num_ctr
    A=[A;eye(num_state)-P(:,:,i)];
    b=[b;G(:,i)];
end
b(find(b==inf))=1e5;
lb = zeros(num_state,1);
ub = Inf(num_state,1);
J_opt = linprog(f,A,b,[],[],lb,ub);

for i=1:num_state
        viablectr = [];
        for k = 1:num_ctr
        	if (sum(P(i,:,k)) > 0)
                viablectr=[viablectr,k];
            end
        end
        [m, ctr_opt] = min( G(i,viablectr) + J_opt'*squeeze(P(i,:,viablectr)) );
        u_opt_ind(i) = viablectr(ctr_opt);
end 
end

