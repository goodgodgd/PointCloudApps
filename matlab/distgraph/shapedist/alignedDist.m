function distance = alignedDist(M, D)

[Ricp Ticp ER t] = icp(M, D, 15, 'Matching', 'kDtree', 'Minimize', 'plane', 'Extrapolation', true);

% Transform data-matrix using ICP result
Dicp = Ricp * D + repmat(Ticp, 1, size(D,2));

% Plot model points blue and transformed points red
figure(2);
subplot(2,2,1);
plot3(M(1,:),M(2,:),M(3,:),'bo',D(1,:),D(2,:),D(3,:),'r.');
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
title('Red: z=sin(x)*cos(y), blue: transformed point cloud');

% Plot the results
subplot(2,2,2);
plot3(M(1,:),M(2,:),M(3,:),'bo',Dicp(1,:),Dicp(2,:),Dicp(3,:),'r.');
axis equal;
xlabel('x'); ylabel('y'); zlabel('z');
title('ICP result');

% Plot RMS curve
subplot(2,2,[3 4]);
plot(0:15,ER,'--x');
xlabel('iteration#');
ylabel('d_{RMS}');
legend('RMS error');
title(['Total elapsed time: ' num2str(t(end),2) ' s']);

k = waitforbuttonpress;
distance = ER(end);
