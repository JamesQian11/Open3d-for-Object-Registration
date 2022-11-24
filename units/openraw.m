clear all;
close all;
clc;
row=240;  
col=180;
%row=200;  
%col=200;

fin=fopen('PhotonIC_1.png','r');
I=fread(fin,[col,row],'uint16=>uint16');
depthVal = 0x7FF;
I = bitand(I,depthVal);
C = I*(255/1800);


%fprintf('d=%d\n', I);
%csvwrite('raw.csv', C);

Z=reshape(C,row,col);
Z = flip(Z,2);


K= imbilatfilt(Z);

r = [0,0,480,450];
%r = centerCropWindow2d (size(K),targetSize);
J = imcrop (K,r);
%csvwrite('raw_60.csv', K);
%imwrite(uint8(J),"raw_1.tif");
%targetSize = [350,480];
%r = centerCropWindow2d (size(K),targetSize);
%J = imcrop (K,r);
%csvwrite('raw_60.csv', K);
%imwrite(uint8(J),"raw_1.tif");

C = imshow(K,[]);
