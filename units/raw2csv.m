clear all;
close all;
clc;
%row=640;  
%col=480;
row=224;  
col=168;
for i = 0:90
    fin = ("p40pro2/"+(i) +".raw");
    fin=fopen(fin,'r');
    I=fread(fin,[col,row],'uint16=>uint16');
    depthVal = 0x1FFF;
    I = bitand(I,depthVal);
    C = I;
    Z=reshape(C,row,col);
    Z = flip(Z,2);

    r = [10,10,140,195];
    J = imcrop (Z,r);
    
    csvwrite("p40pro2/"+(i)+".csv", J);
    imwrite(uint8(J*(255/1500)),"p40pro2/raw_"+(i)+".png");
    K= imshow(J,[]);
    
end