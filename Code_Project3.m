
%% Name:- Sourim Das
%% Assignment - 3
%% This same code can be used for each of the 10 images.
%% Just change the image file name on line 11.
clear
clc

%% Orientation Map

img = imread('3703.pgm');
gaussian_filter2 = fspecial('gaussian',[3,3],2);

[fx, fy] = gradient(gaussian_filter2);
Gx = filter2(fx, img);
Gy = filter2(fy, img);
Gxx = Gx.^2;
Gxy = Gx.*Gy;
Gyy = Gy.^2;
gaussian_filter1 = fspecial('gaussian', [3,3], 1);
Gxx = filter2(gaussian_filter1, Gxx); 
Gxy = filter2(gaussian_filter1, Gxy);
Gyy = filter2(gaussian_filter1, Gyy);


sin2theta = 2*Gxy;
cos2theta = (Gxx-Gyy);

cos2theta = filter2(gaussian_filter2, cos2theta);
sin2theta = filter2(gaussian_filter2, sin2theta);
oriented_image = pi/2 + atan2(sin2theta,cos2theta)/2;
oriented_image1 = atan2(sin2theta,cos2theta)/2;

[rows, cols] = size(oriented_image);
gap = 15;
len = 0.8*gap;
sampled_oriented_reading = oriented_image(gap:gap:rows-gap,gap:gap:cols-gap);

sampled_oriented_reading1 = oriented_image1(gap:gap:rows-gap,gap:gap:cols-gap);

x_offset = len/2*cos(sampled_oriented_reading); y_offset = len/2*sin(sampled_oriented_reading);
subplot(2,1,1);
imshow(img); hold on;title('Orientation Map');
[x,y] = meshgrid(gap:gap:cols-gap, gap:gap:rows-gap);
x = x-x_offset;
y = y-y_offset;
u = x_offset*2;
v = y_offset*2;
quiver(x,y,u,v,0,'.','linewidth',1, 'color','r');
axis equal, axis ij,  hold off

%% Irregularity Calculation Code

[row_in_sample, col_in_sample] = size(sampled_oriented_reading1);
image_matrix = zeros(row_in_sample, col_in_sample);
for i=1:row_in_sample-2
    for j=1:col_in_sample-2
        A=sampled_oriented_reading1(i:i+2,j:j+2);
        numerator=0;
        for x=i:i+2
            for y=j:j+2
                if x == i+1 && y == j+1
                    continue
                else
                    numerator = numerator + A(x-i+1,y-j+1);
                end
            end
        end
        numerator = norm(numerator,'fro');
        denominator=0;
        for m=i:i+2
            for n=j:j+2
                if m == i+1 && n == j+1
                    continue
                else
                    denominator = denominator + norm(A(m-i+1,n-j+1),'fro');
                end
            end
        end
        calculated_irr = 1-(numerator/denominator);
        image_matrix(i+1,j+1)=255 - 255*calculated_irr;
    end
end
irregularity_map_image = mat2gray(image_matrix);
subplot(2,1,2);
imshow(irregularity_map_image); hold off; title('Irregularity Map Image'); 

