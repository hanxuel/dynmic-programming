function y = move(x,action)
if action == 'n'
    y = [x(1),x(2)+1];
elseif action == 'w'
    y = [x(1)-1,x(2)];
elseif action == 'e'
    y = [x(1)+1,x(2)];
elseif action == 's'
    y = [x(1),x(2)-1];
else
    y = x;
end

end