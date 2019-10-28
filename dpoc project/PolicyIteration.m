function [ J_opt, u_opt_ind ] = PolicyIteration( P, G )
%POLICYITERATION Value iteration
%   Solve a stochastic shortest path problem by Policy Iteration.
%
%   [J_opt, u_opt_ind] = PolicyIteration(P, G) computes the optimal cost and
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
%randomly initilize policy
u_opt_ind=ones(num_state,1);
for i=1:num_state
    viable_ctr=find(G(i,:)~=inf);
    u_opt_ind(i)=viable_ctr(end);
end
u_opt_pre=zeros(num_state,1);
J_opt=zeros(num_state,1);
n=0;
while not(isequal(u_opt_ind,u_opt_pre))
    u_opt_pre=u_opt_ind;
    G_temp=zeros(num_state,1);
    P_temp=zeros(num_state,num_state);
    for i=1:num_state
        G_temp(i)=G(i,u_opt_pre(i));
        P_temp(i,:)=P(i,:,u_opt_pre(i));
    end
    %Policy Evaluation
    J_opt=(eye(num_state)-P_temp)\G_temp;  
    for i=1:num_state
        viable_ctr = find(G(i,:)~=inf);
        [m, ctr_opt] = min(G(i,viable_ctr)'+squeeze(P(i,:,viable_ctr))'*J_opt);
        u_opt_ind(i) = viable_ctr(ctr_opt);
    end
end
end

