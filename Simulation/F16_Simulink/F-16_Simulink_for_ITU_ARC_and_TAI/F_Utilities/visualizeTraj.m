function  visualizeTraj(states_list, lats, lons, alts, lat0, lon0, h0, V, faces)
%UNTITLED3 Summary of this function goes here
%   Detailed explanation goes here
ellipsoid = wgs84Ellipsoid;
lats = reshape(lats, 1, []);
lons = reshape(lons, 1, []);
x_array = zeros(length(lons), length(lats));
y_array = zeros(length(lons), length(lats));
z_array = zeros(length(lons), length(lats));

for i=1:length(lats)
    [x_array(:,i),y_array(:,i),z_array(:,i)] = geodetic2ned(lats(i), lons, alts(i,:), lat0, lon0, h0,  ellipsoid) ;
end

figure(1); clf;
s = surf(x_array,y_array,-z_array); hold on; grid on;
xlabel('North');ylabel('East');zlabel('Altitude');
s.EdgeColor = 'none';
plot3(states_list(:,1), states_list(:,2), states_list(:,3))

Kids = figure(1).Children;
Kids(1).YDir = 'reverse';
for i=1:floor(length(states_list(:,1))/20)
    idx = floor((i-1)*20)+1;
    phi_i = -states_list(idx,6) * rad2deg(1);
    theta_i = -states_list(idx,5) * rad2deg(1);
    psi_i = states_list(idx,4) * rad2deg(1);

    vertices = V;
    vertices = transpose(roty(180) * rotx(180) * vertices');
    Rf16 = rotz(psi_i) * roty(theta_i)*rotx(phi_i);
    vertices = transpose(Rf16 * vertices');
    % vertices = transpose(roty(states_list(1,5)*180/pi) * vertices');
    % vertices = transpose(rotz(states_list(1,4)*180/pi) * vertices');
    
    p=patch('faces', faces, 'vertices' ,15*(vertices)+[states_list(idx,1), states_list(idx,2), states_list(idx,3)+8 ]);
    hold on;
end
axis equal
end