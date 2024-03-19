function uvw = ConvertState(alpha,beta,TAS)
% Aircraft Control and Simulation: Lewis page 64 formula 2.3-5
u = TAS*cos(alpha)*cos(beta);
v = TAS*sin(beta);
w = TAS*sin(alpha)*cos(beta);
uvw = [u,v,w];
end