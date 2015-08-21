function my_errorbar(mx,my,ex,ey,pc,lw)

% plotting standard deviation error-bars
np = length(mx);
for i = 1:np
   px1 = plot([mx(i), mx(i)], [my(i)-ey(i),my(i)+ey(i)], [pc,'-']); % std's in y-direction 
   px2 = plot([mx(i)-ex(i), mx(i)+ex(i)], [my(i),my(i)], [pc,'-']); % std's in x-direction 
   set(px1, 'linewidth', lw*0.8);
   set(px2, 'linewidth', lw*0.8);
end

end