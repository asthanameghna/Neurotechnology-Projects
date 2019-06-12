function RingNetworkAnswer
%%%%%%%% Coding the input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = h_input(theta, theta0, c, epsilon)
	out = c * ( 1 - epsilon + epsilon * cos( 2 * (theta - theta0))) ;
end
n_neurons = 50; % number of neurons
pref_angles = (-pi/2: pi/(n_neurons-1) : pi/2)'; % preferred orientation
%Matlab is vectorial so we can work with vectors
h_ext = h_input(pref_angles, 0, 3, 0.1);
%Uncomment if you want to see a plot
%figure; plot(h_ext); xlabel('pref. orentation'); ylabel('h_{ext}')

%%%%% Coding non-linear activiation function %%%%%%%%%%%%%%%%%%%%%%
function out = activation_f(h, T, beta)
    out = (h>T).*(beta*(h - T));
    out = (out<1).*(out-1)+1; 
end
%Uncomment if you want to see a plot
% figure; plot(-15:15,activation_f(-15:15,0,0.1));xlabel('input h'); ylabel('activation function g')

%%%%% Coding neuron model %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = neuron_model(m, h_ext)
	tau = 5;
	threshold = 0;
	b = 0.1;
	out = m + (activation_f(h_ext, threshold, b) - m) / tau ;
end

%%%%%%%% Simulations of the neurons (not connected) %%%%%%%%%%%%%%%%%%%%%
function out = simulation_n(m, theta0, n_neurons, t_steps, epsilon, c)
	out = zeros(n_neurons,t_steps) ;
	pref_angles = (-pi/2: pi/(n_neurons-1) : pi/2)';
	for t=1:1:t_steps
		out(:,t) = m ;
		h_ext = h_input(pref_angles, theta0, c, epsilon);
		m = neuron_model(m, h_ext);
	end
end 


% Run the different simulations
m = zeros(n_neurons,1); % initialisaion of the neuron 
theta0 = 0;
n_neurons = 50 ;
t_steps = 30 ;
epsilon = 0.9;
simul0 = simulation_n(m, theta0, n_neurons, t_steps, epsilon, 1.2) ;
simul1 = simulation_n(m, theta0, n_neurons, t_steps, epsilon, 1.5) ;
simul2 = simulation_n(m, theta0, n_neurons, t_steps, epsilon,  4) ;

%Uncomment if you want to have some plots
% figure;image(simul0*100);xlabel('time'); ylabel('neuron index')
% figure;image(simul1*100);xlabel('time'); ylabel('neuron index')
% figure;image(simul2*100);xlabel('time'); ylabel('neuron index')

%%%%%%%% Implementing the connections %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = connections(n_neurons,  j0,  j2)
	out = zeros(n_neurons, n_neurons);
	pref_angles = (-pi/2: pi/(n_neurons-1) : pi/2)';
	for i=1:1:n_neurons
			out(i,:) = -j0 + j2 * cos(2*(pref_angles(i) - pref_angles));
	end
end

con = connections(50, 86, 150);
%Uncomment if you want to have some plots
% figure; image(con)


%%%%%%%% Simulating connected network %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function out = simulation_c(m, theta0, n_neurons, t_steps, epsilon, c)
	%Simulate a network of neurons
	out = zeros(n_neurons,t_steps) ;
	pref_angles = (-pi/2: pi/(n_neurons-1) : pi/2)';
	con = connections(n_neurons, 86, 112);
	for t=1:1:t_steps
		out(:,t) = m ;
		h_ext = h_input(pref_angles, theta0, c, epsilon);
		m = neuron_model(m, h_ext + con*m);
	end
end

%%%%%%%% Run the different simulations of the ring network %%%%%%%%%%%%%%%%%%%%%%%%%
m = zeros(n_neurons,1);
theta0 = 0;
n_neurons = 50;
t_steps = 30;
simul_c0 = simulation_c(m, theta0, n_neurons, t_steps, 0.1, 1.2) ;
simul_c1 = simulation_c(m, theta0, n_neurons, t_steps, 0.1, 1.5) ;
simul_c2 = simulation_c(m, theta0, n_neurons, t_steps, 0.1, 4) ;

%Uncomment if you want to have some plots
% figure; image(simul_c0*100);xlabel('time'); ylabel('neuron index')
% figure; image(simul_c1*100);xlabel('time'); ylabel('neuron index')
% figure; image(simul_c2*100);xlabel('time'); ylabel('neuron index')


%%%%%%%%%%%%%%%%%% Changing input %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
simul_c3 = simulation_c(m, theta0, n_neurons, t_steps, 0.8, 100) ;
theta1=(2*pi)/3;
simul_change = simulation_c(simul_c3(:,30), theta1, n_neurons, 500,  0.8, 100) ;
% figure; image(simul_c3*100);xlabel('time'); ylabel('neuron index')
% figure; image(simul_change*100);xlabel('time'); ylabel('neuron index')


%%%%%%%%%%%%%%%%%% Removing the stimulation %%%%%%%%%%%%%%%%%%%%%%%
function out = simulation_nostim(m, n_neurons, t_steps)
	out = zeros(n_neurons,t_steps) ;
	con = connections(n_neurons, 86, 112);
	
	for t=1:1:t_steps
		out(:,t) = m ;
		%Changing the first argument of the function
		m = neuron_model(m, con*m);
	end
end 
simul_nostim = simulation_nostim(simul_c0(:,30),  n_neurons, 100) ;
% figure;image(simul_nostim*100);xlabel('time'); ylabel('neuron index')
end