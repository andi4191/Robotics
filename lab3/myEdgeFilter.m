
function [Ix, Iy, Im, Io]=myEdgeFilter(img, sigma)

[rw,cl,junk]=size(img);
if(junk>=3)
    img=rgb2gray(img);
end

Io=zeros(size(img));

%Gaussian SOBEL Filters
Gxk=[-1,0,1;-2,0,2;-1,0,1];
Gyk=[1,2,1;0,0,0;-1,-2,-1];

kx=2*ceil(2*sigma)+1;
%kx=floor(3*sigma);
%kx=3;
ky=kx;

[k1,k2]=meshgrid(-(kx-1)/2:(kx-1)/2,-(ky-1)/2:(ky-1)/2);
gaussk=exp(-(k1.^2+k2.^2)/(2*sigma^2));
gaussker=gaussk./sum(gaussk(:));

img1=myImageFilter(img,gaussker);
%figure();
%imshow(img1);

tempImg=double(img1);
[r,c]=size(img1);
out1=zeros(size(img1));
for i=2:r-1
    for j=2:c-1
        imggrid=tempImg((i-1):(i+1),(j-1):(j+1));
        out1(i,j)=sum(sum(Gxk.*imggrid));
    end
end

%figure();
Ix=uint8(out1);
%Ix=double(out1);
%imshow(Ix);

out2=zeros(size(img1));
for i=2:r-1
    for j=2:c-1
        imggrid=tempImg((i-1):(i+1),(j-1):(j+1));
        out2(i,j)=sum(sum(Gyk.*imggrid));
    end
end
%figure();
Iy=uint8(out2);
%Iy=double(out2);
%imshow(Iy);

for i=2:r-1
    for j=2:c-1
        if(Ix(i,j)==0)
            k=0.25;
        else
            k=Ix(i,j);
        end
        Io(i,j)=atan2(double(Iy(i,j)),double(k));
    end
end


%figure();
Im=sqrt(out1.^2+out2.^2);
%imshow(Im);

%figure();
%imshow(Io);
[rn,cn]=size(Im);
for i=2:rn-1
    for j=2:cn-1
        deg=Io(i,j);
        if(abs(deg)>=0 && ((abs(deg)<=pi/8) || abs(deg-pi)<=pi/8)) %Check for vertical neigbours
            if(Im(i-1,j)>Im(i,j) || Im(i+1,j)>Im(i,j))
                Im(i,j)=0;
            end
        elseif((abs(deg-pi/2)>=0 && abs(deg-pi/2)<=pi/8) || (abs(deg-3*pi/2)>=0 && abs(deg-3*pi/2)<=pi/8)) %Check for horizontal neighbours
            if(Im(i,j-1)>Im(i,j) || Im(i,j+1)>Im(i,j))
                Im(i,j)=0;
            end
        elseif((abs(deg-pi/4)>=0 && abs(deg-pi/4)<=pi/8) || (abs(deg-5*pi/4)>=0 && abs(deg-5*pi/4)<=pi/8))  %Check for diagonal neighbours
            if(Im(i-1,j-1)>Im(i,j) || Im(i+1,j+1)>Im(i,j))
                Im(i,j)=0;
            end
        
        elseif((abs(deg-7*pi/4)>=0 && abs(deg-7*pi/4)<=pi/8) || (abs(deg-3*pi/4)>=0 && abs(deg-3*pi/4)<=pi/8))  %Check for anti-diagonal neighbours
            if(Im(i+1,j+1)>Im(i,j) || Im(i-1,j-1)>Im(i,j))
                Im(i,j)=0;
            end
        end
    end

Im=uint8(Im);


end