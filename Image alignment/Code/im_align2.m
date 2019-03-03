function im_align2(R_cut,G_cut,B_cut,offset,R,G,B,number)
        [b_x,b_y] = align2(B_cut, G_cut, offset);
        Moved_B = circshift(B, [b_y, b_x]);
        
        [r_x,r_y] = align2(R_cut, G_cut, offset);
        Moved_R = circshift(R, [r_y, r_x]);
        result = cat(3, Moved_R, G, Moved_B);
        imwrite(result,['image' num2str(number) '-ncc.jpg']);
end

function [x, y] = align2(Moving_image, background, offset)
    backgroundV = background(:);
    displacement = zeros((offset*2));
    backgroundNorm = backgroundV/norm(backgroundV);
    
    for h = -offset+1:offset
        for w = -offset+1:offset
            Moving_image_Shifted = circshift(Moving_image,[h,w]);
            x1 = w+offset;
            y1 = h+offset;
            Moving_imageV = Moving_image_Shifted(:);
            Moving_imageNorm = Moving_imageV/norm(Moving_imageV);
            displacement(y1, x1) = dot(backgroundNorm, Moving_imageNorm);

        end
    end
    [Max,Index] = max(displacement(:));
    [y, x] = ind2sub(size(displacement),Index);
    x = x-offset;
    y = y-offset;

end