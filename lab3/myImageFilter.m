
function [img1]=myImageFilter(img0, h)

img=img0;
%Dimension of filter
[kx,ky]=size(h);

%Dimension of image
[r,c]=size(img);

%For padding 
r=r+2*floor(kx/2);
c=c+2*floor(ky/2);

img2=uint8(zeros(r,c));
fr=ceil(kx/2);
lr=(r-floor(kx/2));
fc=ceil(ky/2);
lc=(c-floor(ky/2));
for i=fr:lr
    for j=fc:lc
        img2(i,j)=img(i-fr+1,j-fc+1);
    end
end


%For padding with boundary pixel values
frow=img2(fr,:);
lrow=img2(lr,:);

fcol=img2(:,fc);
lcol=img2(:,lc);
for i=1:floor(kx/2)
    img2(i,:)=frow;
    img2(r-i+1,:)=lrow;
end
for i=1:floor(ky/2)
    img2(:,i)=fcol;
    img2(:,c-i+1)=lcol;
end


if(sum(sum(h))==0)
    gradVal=1;
else
    gradVal=sum(sum(h));
end
[rh,ch]=size(h);
flipped_h=zeros(rh,ch);
for i=1:rh
    for j=1:ch
        tar_r=rh-i+1;
        tar_c=ch-j+1;
        flipped_h(i,j)=h(tar_r,tar_c);
    end
end

h=flipped_h;
%h
%flipped_h
tempImg=double(img2);

for i=fr:lr
    for j=fc:lc
        grid=tempImg((i-fr+1):(i+fr-1),(j-fc+1):(j+fc-1));
        img2(i,j)=sum(sum(h.*grid));
    end
end     

img1=img2(fr:lr,fc:lc);
%figure();
%imshow(img1);
end

