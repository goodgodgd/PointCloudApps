clear
clc

data = load('data.mat');
data = data.data;

radii = [4 5 6];
gradw = [0.2 0.3 0.4 0.5];
meanData = zeros(length(radii), length(gradw));

for r=1:length(radii)
    radInd = data(:,2)==radii(r);
    meanData(r,:) = mean(data(radInd,3:6), 1);
end
meanData

plot(gradw, meanData(1,:), 'o-')
hold on
plot(gradw, meanData(2,:), 'x-')
plot(gradw, meanData(3,:), '^-')
hold off
legend('r=4cm', 'r=5cm', 'r=6cm')
axis([0.2 0.6 0.15 0.3])
xlabel('gradient weight')