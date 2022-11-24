clc;
clear all;

% 读取所有nii后缀的文件
fin = dir('*.raw');
for i=1:length(fin)    
    old_name=fin(i).name;%取出第一个文件的名称         
    new_name = strcat('depth_', int2str(i), '.raw');
    movefile(old_name,new_name)
end