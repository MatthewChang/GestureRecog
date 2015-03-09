V = imread('vf197.657.jpg');
M = imread('mf197.657.bmp');
sv = size(V);
sm = size(M);
datasize = [sv(1)*sv(2),sv(3)];
masksize = [sv(1)*sv(2),1];
M(M>1) = 1;
imMask = M;
imMask(:,:,2) = imMask(:,:,1);
imMask(:,:,3) = imMask(:,:,1);
masked = V.*imMask;
imshow(masked);
shaped = double(reshape(V,datasize));
shaped_mask = reshape(M,masksize);
size(shaped);
size(shaped_mask);
B = TreeBagger(20,shaped,shaped_mask,'OOBPred','on');

oobErrorBaggedEnsemble = oobError(B);
plot(oobErrorBaggedEnsemble)
xlabel 'Number of grown trees';
ylabel 'Out-of-bag classification error';

in2 = imread('vf517.272.jpg');
in2shape = double(reshape(in2,datasize));
Y = predict(B,in2shape);
Y = str2double(Y);
R = double(reshape(Y,sm));
imshow(R)