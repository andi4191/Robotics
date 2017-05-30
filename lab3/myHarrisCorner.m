function [R] = myHarrisCorner(Ix, Iy, threshold)

sigma=0.5;
Ix2=Ix.^2;
Iy2=Iy.^2;
Ixy=Ix.*Iy;

kx=2*ceil(2*sigma)+1;
%kx=3;
ky=kx;

[k1,k2]=meshgrid(-(kx-1)/2:(kx-1)/2,-(ky-1)/2:(ky-1)/2);
gaussk=exp(-(k1.^2+k2.^2)/(2*sigma^2));
gaussker=gaussk./sum(gaussk(:));


Ix2=myImageFilter(Ix2,gaussker);
Iy2=myImageFilter(Iy2,gaussker);
Ixy=myImageFilter(Ixy,gaussker);

k=0.04;
[r,c]=size(Ix);
M=zeros(2,2);
R=zeros(r,c);
int_res=double(zeros(r,c));
for i=1:r
    for j=1:c
        res=0;
        M(1,1)=Ix2(i,j);
        M(1,2)=Ixy(i,j);
        M(2,1)=Ixy(i,j);
        M(2,2)=Iy2(i,j);
        detr=det(M);
        trace2=trace(M).^2;
        res=detr-k*trace2;
        int_res(i,j)=res;
        if(res>threshold)
            R(i,j)=1;
        end
    end
end

%int_res

max(int_res(:))
end





