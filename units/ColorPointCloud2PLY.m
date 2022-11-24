function ColorPointCloud2PLY(X, Y, Z, IsColor, PLYfileName2write)
% ColorPointCloud2PLY(X,Y,Z,IsColor, PLYfileName2write)
% ColorPointCloud2PLY writes range grid data with a colormap to a PLY file
% X,Y,Z are 3 vectors representing Point Cloud
% PLYfileName2write: path/name of the output ply file (e.g. 'dir/fname.ply')
% ColorMap is based on Z depths
% JET colormap is used, but can be changed to any other preferable map
% IsColor = 0/1 - choose whether to write color or plain PLY file
%
% Author: Avishay Assayag at Inuitive-tech.com, March 2017
% ColorMatrix is a matrix of size Mx3 (Same dimensions as Point Cloud) contains RGB values for each point on Point Cloud
if IsColor
    FigureHandle = figure('visible', 'off');
    ColorMap = colormap(jet(length(unique(Z))));
    ColorMatrix = zeros(length(Z),3);
    Zsorted=sort(unique(Z));
    h = waitbar(0,'Please wait...');
    for k = 1:length(Zsorted)
        Ind = find(Z==Zsorted(k));
        ColorMatrix(Ind,:)=repmat(ColorMap(k,1:3),length(Ind),1);
        waitbar(k/length(Zsorted),h)
    end
    close(h)
    ColorMatrix=round(255*ColorMatrix);
else
    ColorMatrix = [];
end
X=X(:);
Y=Y(:);
Z=Z(:);
if ~isempty(ColorMatrix)
    R = ColorMatrix(:,1);
    G = ColorMatrix(:,2);
    B = ColorMatrix(:,3);
end
M = (X(:)==0)&(Y(:)==0)&(Z(:)==0);
fid = fopen(PLYfileName2write,'w');
fprintf(fid,'ply\n');
fprintf(fid,'format ascii 1.0\n');
fprintf(fid,'obj_info num_cols %d\n',size(X,1));
fprintf(fid,'obj_info num_rows %d\n',size(X,2));
fprintf(fid,'element vertex %d\n',sum(~M));
fprintf(fid,'property float x\n');
fprintf(fid,'property float y\n');
fprintf(fid,'property float z\n');
if ~isempty(ColorMatrix)
    fprintf(fid,'property uchar red\n');
    fprintf(fid,'property uchar green\n');
    fprintf(fid,'property uchar blue\n');
end
fprintf(fid,'element range_grid %d\n',numel(X));
fprintf(fid,'property list uchar int vertex_indices\n');
fprintf(fid,'end_header\n');
if isempty(ColorMatrix)
    fprintf(fid,'%f %f %f\n',[X(~M) Y(~M) Z(~M)]');
else
    fprintf(fid,'%f %f %f %.0f %.0f %.0f\n',[X(~M) Y(~M) Z(~M) R(~M) G(~M) B(~M)]');
end
fprintf(fid,'%d %d\n',[~M (~M).*cumsum(~M)]');
fclose(fid);

