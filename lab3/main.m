function main()

clear all
clc
img_1='img01.jpg';
img_2='img02.jpg';
img_3='img03.jpg';
img_4='img04.jpg';
img_5='img05.jpg';
imags={img_1,img_2,img_3,img_4,img_5};

sigma=0.5;

threshold=4500;

[lr,lc]=size(imags);

for i=1:lc
    imgs=char(imags(i));
    
    img1=imread(imgs);
    img=img1;
    
    [rw,cl,junk]=size(img1);
    if(junk>=3)
        img=rgb2gray(img1);
    else
        img=img1;
    end
    
    
    [Ix,Iy,Im,Io]=myEdgeFilter(img,1);%sigma);

    R=myHarrisCorner(Ix,Iy,threshold);
    

    [rows,cols]=find(R);
    
    fig=figure();
    imshow(img);
    hold on
    plot(cols,rows,'ro');
    t=datetime('now');
    st=strsplit(char(t),' ');
    fn=strcat(st(1),'_');
    fn=strcat(fn,st(2));
    fname=strcat(fn,imags(i));
    
    saveas(fig,char(fname));
    
    %hold off
  
end
end
