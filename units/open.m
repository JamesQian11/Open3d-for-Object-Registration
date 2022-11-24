clear all;
close all;
clc;
% %row=640;  
% %col=480;
% row=224;  
% col=168;
% 
% fin=fopen('HW.raw','r');
% I=fread(fin,[col,row],'uint16=>uint16');
% depthVal = 0x7FF;
% I = bitand(I,depthVal);
% C = I*(255/1800);
% 
% %fprintf('d=%d\n', I);
% %csvwrite('raw.csv', C);
% 
% Z=reshape(C,row,col);
% Z = flip(Z,2);
% 
% 
% K= imbilatfilt(Z);
% 
% %r = [0,0,480,450];
% %r = centerCropWindow2d (size(K),targetSize);
% %J = imcrop (K,r);
% %csvwrite('raw_60.csv', K);
% %imwrite(uint8(J),"raw_1.tif");
% %targetSize = [350,480];
% %r = centerCropWindow2d (size(K),targetSize);
% %J = imcrop (K,r);
% csvwrite('HW.csv', K);
% %imwrite(uint8(J),"raw_1.tif");
% 
% C = imshow(K,[]);

row=224;  
col=168;
fin=fopen('p40pro2/30.raw','r');
I=fread(fin,[col,row],'uint16=>uint16');
depthVal = 0x1FFF;
I = bitand(I,depthVal);
C = I;
Z=reshape(C,row,col);
Z = flip(Z,2);
%K= imbilatfilt(Z);
r = [10,10,140,195];
J = imcrop (Z,r);
%csvwrite('1.csv', J);
%imwrite(uint8(J),"test1.png");
C = imshow(J,[]);