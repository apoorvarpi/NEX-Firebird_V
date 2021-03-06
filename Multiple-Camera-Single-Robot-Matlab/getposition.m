function [x,y]=getposition(obstacle,robot,goal,curr_x,curr_y)
    dt=20;
    force_att_x=goal(1)-curr_x;
    force_att_y=goal(2)-curr_y;
    force_rep_x=0;
    force_rep_y=0;
    curr_pos=[curr_x,curr_y];
    eps=50;
    neta=4;
    no_of_obstacles=numel(obstacle)/2;
    for i=1:no_of_obstacles
        d=dist(obstacle,i,curr_pos);
        if(d<eps)
            force_rep_x=force_rep_x+neta*(curr_x-obstacle(i,1));
            force_rep_y=force_rep_y+neta*(curr_y-obstacle(i,2));
        end
    end
%    for i=1:numel(robot)/2
%        d=dist(robot,i,curr_pos);
%        if(d<eps)
%            force_rep_x=force_rep_x+(curr_x-robot(i,1));
%            force_rep_y=force_rep_y+(curr_y-robot(i,2));
%        end
%    end
    force_rep_x;
    force_rep_y;
    force_att_x;
    force_att_y;
    force_net_x=force_rep_x+force_att_x;
    force_net_y=force_rep_y+force_att_y;
    theta=atan2(force_net_y,force_net_x);
    
    if(sqrt(force_net_y*force_net_y+force_net_x*force_net_x)==0)
        x=curr_x;
        y=curr_y;
   %     theta=theta+pi/2;  
    end
    x=curr_x+cos(theta)*dt;
    y=curr_y+sin(theta)*dt;
end