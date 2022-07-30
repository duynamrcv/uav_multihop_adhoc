%
% Create random position (solutions)
% 

function sol=CreateRandomSolution(VarMin,VarMax) 
    sol.x=unifrnd(VarMin.x,VarMax.x);
    sol.y=unifrnd(VarMin.y,VarMax.y);
    sol.z=unifrnd(VarMin.z,VarMax.z);
end