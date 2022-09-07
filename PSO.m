function [newPoint, BestCost] = PSO(start,currentState,model,VarMax,VarMin)

    %% PSO parameter
    MaxIt = 100;
    nPop=50;           % Population Size (Swarm Size)

    w=1;                % Inertia Weight
    wdamp=0.98;         % Inertia Weight Damping Ratio
    c1=1.5;             % Personal Learning Coefficient
    c2=1.5;             % Global Learning Coefficient
    
    % Lower and upper Bounds of velocity
    alpha=0.5;
    VelMax.x=alpha*(VarMax.x-VarMin.x);    
    VelMin.x=-VelMax.x;                    
    VelMax.y=alpha*(VarMax.y-VarMin.y);    
    VelMin.y=-VelMax.y;                    
    VelMax.z=alpha*(VarMax.z-VarMin.z);    
    VelMin.z=-VelMax.z; 
    
    % Initialization
    % Create Empty Particle Structure
    empty_particle.Position=[];
    empty_particle.Velocity=[];
    empty_particle.Cost=[];
    empty_particle.Best.Position=[];
    empty_particle.Best.Cost=[];
    
    % TargetInfor.distance, TargetInfor.direction
%     TargetInfor = MoveToTarget(currentState.Position,target); 
    target.x = model.goal(1);
    target.y = model.goal(2);
    target.z = model.goal(3);

    CostFunction=@(x) MyCost(x,start,currentState,target,model,VarMax,VarMin);
    
    % Initialize Global Best
    GlobalBest.Cost=inf; % Minimization problem

    % Create an empty Particles Matrix, each particle is a solution (searching path)
    particle=repmat(empty_particle,nPop,1);

    % Initialization Loop
    isInit = false;
    while (~isInit)
%             disp('Initialising...');
       for i=1:nPop

            % Initialize Position
            particle(i).Position=CreateRandomSolution(VarMin,VarMax);

            % Initialize Velocity
            particle(i).Velocity.x=0;
            particle(i).Velocity.y=0;
            particle(i).Velocity.z=0;

            % Evaluation
            particle(i).Cost= CostFunction(particle(i).Position);

            % Update Personal Best
            particle(i).Best.Position=particle(i).Position;
            particle(i).Best.Cost=particle(i).Cost;

            % Update Global Best
            if particle(i).Best.Cost < GlobalBest.Cost
                GlobalBest=particle(i).Best;
                isInit = true;
            end
        end
    end
    
    % PSO loop
    for it=1:MaxIt
        for i=1:nPop          
            % x Part
            % Update Velocity
            particle(i).Velocity.x = w*particle(i).Velocity.x ...
                + c1*rand().*(particle(i).Best.Position.x-particle(i).Position.x) ...
                + c2*rand().*(GlobalBest.Position.x-particle(i).Position.x);

            % Update Velocity Bounds
            particle(i).Velocity.x = max(particle(i).Velocity.x,VelMin.x);
            particle(i).Velocity.x = min(particle(i).Velocity.x,VelMax.x);

            % Update Position
            particle(i).Position.x = particle(i).Position.x + particle(i).Velocity.x;

            % Velocity Mirroring
            % If a particle moves out of the range, it will moves backward next
            % time
            OutOfTheRange=(particle(i).Position.x<VarMin.x | particle(i).Position.x>VarMax.x);
            particle(i).Velocity.x(OutOfTheRange)=-particle(i).Velocity.x(OutOfTheRange);

            % Update Position Bounds
            particle(i).Position.x = max(particle(i).Position.x,VarMin.x);
            particle(i).Position.x = min(particle(i).Position.x,VarMax.x);


            % y Part

            % Update Velocity
            particle(i).Velocity.y = w*particle(i).Velocity.y ...
                + c1*rand().*(particle(i).Best.Position.y-particle(i).Position.y) ...
                + c2*rand().*(GlobalBest.Position.y-particle(i).Position.y);

            % Update Velocity Bounds
            particle(i).Velocity.y = max(particle(i).Velocity.y,VelMin.y);
            particle(i).Velocity.y = min(particle(i).Velocity.y,VelMax.y);

            % Update Position
            particle(i).Position.y = particle(i).Position.y + particle(i).Velocity.y;

            % Velocity Mirroring
            OutOfTheRange=(particle(i).Position.y<VarMin.y | particle(i).Position.y>VarMax.y);
            particle(i).Velocity.y(OutOfTheRange)=-particle(i).Velocity.y(OutOfTheRange);

            % Update Position Bounds
            particle(i).Position.y = max(particle(i).Position.y,VarMin.y);
            particle(i).Position.y = min(particle(i).Position.y,VarMax.y);

            % z part
            % Update Velocity
            particle(i).Velocity.z = w*particle(i).Velocity.z ...
                + c1*rand().*(particle(i).Best.Position.z-particle(i).Position.z) ...
                + c2*rand().*(GlobalBest.Position.z-particle(i).Position.z);

            % Update Velocity Bounds
            particle(i).Velocity.z = max(particle(i).Velocity.z,VelMin.z);
            particle(i).Velocity.z = min(particle(i).Velocity.z,VelMax.z);

            % Update Position
            particle(i).Position.z = particle(i).Position.z + particle(i).Velocity.z;

            % Velocity Mirroring
            OutOfTheRange=(particle(i).Position.z<VarMin.z | particle(i).Position.z>VarMax.z);
            particle(i).Velocity.z(OutOfTheRange)=-particle(i).Velocity.z(OutOfTheRange);

            % Update Position Bounds
            particle(i).Position.z = max(particle(i).Position.z,VarMin.z);
            particle(i).Position.z = min(particle(i).Position.z,VarMax.z);

            % Evaluation
            particle(i).Cost=CostFunction(particle(i).Position);

            % Update Personal Best
            if particle(i).Cost < particle(i).Best.Cost

                particle(i).Best.Position=particle(i).Position;
                particle(i).Best.Cost=particle(i).Cost;

                % Update Global Best
                if particle(i).Best.Cost < GlobalBest.Cost
                    GlobalBest=particle(i).Best;
                end

            end

        end
        % Inertia Weight Damping
        w=w*wdamp;

        % Update Best Cost Ever Found
        BestCost(it)=GlobalBest.Cost;
        
        % Show Iteration Information
        disp(['Iteration ' num2str(it) ': Best Cost = ' num2str(BestCost(it))]);

    end

    newPoint = [GlobalBest.Position.x,...
                GlobalBest.Position.y,...
                GlobalBest.Position.z];
end