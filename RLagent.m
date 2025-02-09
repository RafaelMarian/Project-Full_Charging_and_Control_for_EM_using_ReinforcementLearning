clear;%clear the variables
clc;%clear the command window

% Data
PWM_frequency 	= 1e3;%PWM motor frequency set to 1000Hz             
T_pwm           = 1/PWM_frequency;%Period of the Frequency 
Ts = T_pwm; %Sampling time
Ts_simulink     = T_pwm/2;%Sampling time
Ts_speed = 10*Ts;  %Sampling time  
dataType = 'single';           

% Simulink Model
mdl = 'DisertatieVanca';
open_system(mdl)

% Create observation specifications. (Defines an 8 dimensional input space,
% including information such as motor error, speed and reference signals.
numObservations = 8;%Dimension of the observation space
observationInfo = rlNumericSpec([numObservations 1]);%Observations are numeric with additonal name and descriptions
observationInfo.Name = 'observations';
observationInfo.Description = 'Information on error and reference signal';

% Create action specifications.
numActions = 2;%defines the dimension of the action space
actionInfo = rlNumericSpec([numActions 1]); 
actionInfo.Name = 'vqdRef';

agentblk = 'DisertatieVanca/Reinforcement Learning/RL Agent';%Specifies the block in Simulink
env = rlSimulinkEnv(mdl,agentblk,observationInfo,actionInfo);%Creates an reinforced learning environment that connects Simulink with actions and observations

% Create Agent
rng(0)  % fix the random seed 
%State Path: Processes observation inputs through a dense layer.
%Action Path: Processes action inputs through another dense layer.
%Common Path: Combines the processed state and action paths, applies activation functions, and outputs a single value (Q-value).

statePath = [featureInputLayer(numObservations,'Normalization','none','Name','State')
    fullyConnectedLayer(64,'Name','fc1')];
actionPath = [featureInputLayer(numActions, 'Normalization', 'none', 'Name','Action')
    fullyConnectedLayer(64, 'Name','fc2')];
commonPath = [additionLayer(2,'Name','add')
    reluLayer('Name','relu2')
    fullyConnectedLayer(32, 'Name','fc3')
    reluLayer('Name','relu3')
    fullyConnectedLayer(16, 'Name','fc4')
    fullyConnectedLayer(1, 'Name','CriticOutput')];

criticNetwork = layerGraph();
criticNetwork = addLayers(criticNetwork,statePath);
criticNetwork = addLayers(criticNetwork,actionPath);
criticNetwork = addLayers(criticNetwork,commonPath);
criticNetwork = connectLayers(criticNetwork,'fc1','add/in1');
criticNetwork = connectLayers(criticNetwork,'fc2','add/in2');

criticOptions = rlRepresentationOptions('LearnRate',1e-4,'GradientThreshold',1);
critic1 = rlQValueRepresentation(criticNetwork,observationInfo,actionInfo,...
    'Observation',{'State'},'Action',{'Action'},criticOptions);
critic2 = rlQValueRepresentation(criticNetwork,observationInfo,actionInfo,...
    'Observation',{'State'},'Action',{'Action'},criticOptions);


actorNetwork = [featureInputLayer(numObservations,'Normalization','none','Name','State')
    fullyConnectedLayer(64, 'Name','actorFC1')
    reluLayer('Name','relu1')
    fullyConnectedLayer(32, 'Name','actorFC2')
    reluLayer('Name','relu2')
    fullyConnectedLayer(numActions,'Name','Action')
    tanhLayer('Name','tanh1')];
actorOptions = rlRepresentationOptions('LearnRate',1e-3,'GradientThreshold',1,'L2RegularizationFactor',0.001);
actor = rlDeterministicActorRepresentation(actorNetwork,observationInfo,actionInfo,...
    'Observation',{'State'},'Action',{'tanh1'},actorOptions);

%Create DDPG Agent
Ts_agent = Ts;
agentOptions = rlTD3AgentOptions("SampleTime",Ts_agent, ...
    "DiscountFactor", 0.995, ...
    "ExperienceBufferLength",2e6, ...
    "MiniBatchSize",512, ...
    "NumStepsToLookAhead",1, ...
    "TargetSmoothFactor",0.005, ...
    "TargetUpdateFrequency",10);

agentOptions.ExplorationModel.Variance = 0.05;
agentOptions.ExplorationModel.VarianceDecayRate = 2e-4;
agentOptions.ExplorationModel.VarianceMin = 0.001;

agentOptions.TargetPolicySmoothModel.Variance = 0.1;
agentOptions.TargetPolicySmoothModel.VarianceDecayRate = 1e-4;

agent = rlTD3Agent(actor,[critic1,critic2],agentOptions);

%% Train Agent
T = 5.0;%Total training durations
maxepisodes = 1000;%Limit how many training episodes can occur
maxsteps = ceil(T/Ts_agent); %Defines the maximum steps per episode based on sampling time
trainingOpts = rlTrainingOptions(...
    'MaxEpisodes',maxepisodes, ...
    'MaxStepsPerEpisode',maxsteps, ...
    'StopTrainingCriteria','AverageReward',...
    'StopTrainingValue',-190,... 
    'ScoreAveragingWindowLength',100);

%% Simualation
sim(mdl)