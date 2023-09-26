function ExportStereoParamsOpenCV(stereoParams, file)
% ExportStereoParams2OpenCV(cameraParams, file)
% 功能：将双目相机标定的参数保存为OpenCV可读取的xml文件
% 输入：
%   stereoParams：双目相机标定数据的对象
%   file：输出的xml文件名
% xml文件是由一层层的节点组成的。
% 首先创建父节点 fatherNode，
% 然后创建子节点 childNode=docNode.createElement(childNodeName)，
% 再将子节点添加到父节点 fatherNode.appendChild(childNode)
 
%创建xml文件对象，添加opencv存储标识
doc = com.mathworks.xml.XMLUtils.createDocument('opencv_storage');
docRootNode = doc.getDocumentElement; %获取根节点
 
M1 = (stereoParams.CameraParameters1.IntrinsicMatrix).'; % 右相机内参矩阵，需要装置为左乘的矩阵
RadialDistortion1 = stereoParams.CameraParameters1.RadialDistortion; % 右相机径向畸变参数向量1*3
TangentialDistortion1 =stereoParams.CameraParameters1.TangentialDistortion; % 右相机切向畸变向量1*2
D1 = zeros(1,5); % OpenCV的畸变参数矩阵最多可以有14个参数
D1(1:4) = [RadialDistortion1(1:2), TangentialDistortion1]; %构成opencv中的畸变系数向量[k1,k2,p1,p2,k3]
if (size(RadialDistortion1,2)>2 )
    D1(5) = RadialDistortion1(3);
end
M2 = (stereoParams.CameraParameters2.IntrinsicMatrix).'; % 右相机内参矩阵，需要装置为左乘的矩阵
RadialDistortion2 = stereoParams.CameraParameters2.RadialDistortion; % 右相机径向畸变参数向量1*3
TangentialDistortion2 =stereoParams.CameraParameters2.TangentialDistortion; % 右相机切向畸变向量1*2
D2 = zeros(1,5);
D2(1:4) = [RadialDistortion2(1:2), TangentialDistortion2]; %构成opencv中的畸变系数向量[k1,k2,p1,p2,k3]
if (size(RadialDistortion1,2)>2 )
    D2(5) = RadialDistortion2(3);
end
 
R = (stereoParams.RotationOfCamera2).';
T = (stereoParams.TranslationOfCamera2).';
sizeImage = stereoParams.CameraParameters1.ImageSize;
 
AddMat(sizeImage, 'size', doc, docRootNode);
AddMat(M1, 'M1', doc, docRootNode);
AddMat(D1, 'D1', doc, docRootNode);
AddMat(M2, 'M2', doc, docRootNode);
AddMat(D2, 'D2', doc, docRootNode);
AddMat(R, 'R', doc, docRootNode);
AddMat(T, 'T', doc, docRootNode);
 
xmlwrite(file, doc);
 
end
 
function AddMat(A, name, doc, docRootNode)
 
mat = doc.createElement(name); %创建mat节点
 
mat.setAttribute('type_id','opencv-matrix'); %设置mat节点属性
 
rows = doc.createElement('rows'); %创建行节点
rows.appendChild(doc.createTextNode(sprintf('%d',size(A,1)))); %创建文本节点，并作为行的子节点
mat.appendChild(rows); %将行节点作为mat子节点
 
cols = doc.createElement('cols');
cols.appendChild(doc.createTextNode(sprintf('%d',size(A,2))));
mat.appendChild(cols);
 
dt = doc.createElement('dt');
dt.appendChild(doc.createTextNode('d'));
mat.appendChild(dt);
 
data = doc.createElement('data');
for i=1:size(A,1)
    for j=1:size(A,2)
        data.appendChild(doc.createTextNode(sprintf('%.15f ',A(i,j))));
    end
    data.appendChild(doc.createTextNode(sprintf('\n')));
end
mat.appendChild(data);
docRootNode.appendChild(mat);
 
end