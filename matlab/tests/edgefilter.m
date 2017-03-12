%% Find Vertical and Horizontal Edges in Image
% Construct Haar-like wavelet filters to find vertical and horizontal edges in an image.
%%
% Read the input image and compute the integral image.

% Copyright 2015 The MathWorks, Inc.

close all

I = imread('pout.tif');
intImage = integralImage(I);
%%
% Construct Haar-like wavelet filters. Use the dot notation to find the vertical filter from the horizontal filter.
vertH = integralKernel([1 1 4 3; 1 4 4 3],[-1, 1]);
horiH = vertH.'
horiH.Coefficients
%%
% Display the horizontal filter.
% imtool(horiH.Coefficients, 'InitialMagnification','fit');
%%
% Compute the filter responses.
horiResponse = integralFilter(intImage,horiH);
vertResponse = integralFilter(intImage,vertH);
%%
% Display the results.
figure; 
imshow(horiResponse,[]);
title('Horizontal edge responses');
figure; 
imshow(vertResponse,[]); 
title('Vertical edge responses');