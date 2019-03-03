
for i = 1:6
    first(['image' num2str(i) '.jpg'],i);
end
function first(name,number)
    %read image
    image = imread(name);
    image = im2double(image);
    length = floor(size(image,1)/3);
    width  = floor(size(image,2));
    
    %seprate the RGB image into 3 BGR images
    R = image(length*2+1:length*3,:);
    G = image(length+1:length*2,:);
    B = image(1:length,:);
    
    %cut image
    a = ((length+width)/2)*(0.10);
    %image after cut
    R_cut = R(1+a:length-a,1+a:width-a);
    G_cut = G(1+a:length-a,1+a:width-a);
    B_cut = B(1+a:length-a,1+a:width-a);
    %Unaligned pictures
    Output = cat(3,R,G,B);
    imwrite(Output,['image' num2str(number) '-color.jpg'])
    im_align1(R_cut,G_cut,B_cut,15,R,G,B,number);
    im_align2(R_cut,G_cut,B_cut,15,R,G,B,number);
    
end