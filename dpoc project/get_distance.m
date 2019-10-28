function d = get_distance(y1,y2)
    if y1(1) == y2(1)
        d = abs(y2(2)-y1(2));
    else
        d = abs(y2(1)-y1(1));
    end
    
end