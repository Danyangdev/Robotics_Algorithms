function im_align1(R_cut,G_cut,B_cut,offset,R,G,B,number)
        [b_x,b_y] = align1(B_cut, G_cut, offset);
        Moved_B = circshift(B, [b_y, b_x]);
        
        [r_x,r_y] = align1(R_cut, G_cut, offset);
        Moved_R = circshift(R, [r_y, r_x]);
        result = cat(3, Moved_R, G, Moved_B);
        imwrite(result,['image' num2str(number) '-ssd.jpg']);
end
function [x, y] = align1(Moving_image, background, offset)
    displacement = zeros((offset*2));
    for l = -offset+1:offset
        for w = -offset+1:offset
            Moving_image_Shifted = circshift(Moving_image,[l,w]);
            x1 = w+offset;
            y1 = l+offset;
            displacement(y1, x1) = sum(sum((background-Moving_image_Shifted).^2));
        end
    end
    [Max,Index] = min(displacement(:));
    [y, x] = ind2sub(size(displacement), Index);
    x = x-offset;
    y = y-offset;

end
