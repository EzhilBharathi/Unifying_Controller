%{
Implenting the paper:Unifying Geometric, Probabilistic, and Potential Field Approaches
to Multi-Robot Deployment 
%}

clear
clf
%randomly generating agent positions 
pointset1= (rand(1,2)+1)*1.5;pointset2 =(rand(1,2)+1)*1.5;pointset3 = (rand(1,2)+1)*1.5;pointset4 =(rand(1,2)+1)*1.5;pointset11= (rand(1,2)+1)*1.5;
pointset12 =(rand(1,2)+1)*1.5;pointset13 =(rand(1,2)+1)*1.5;pointset14 =(rand(1,2)+1)*1.5;pointset21=(rand(1,2)+1)*1.5;
%concatinating agent positions into single matrix
allagents2=cat(1,pointset1,pointset2,pointset3,pointset4,pointset11,pointset12,pointset13);
allagents1=cat(1,allagents2,pointset14,pointset21);
allagents=cat(1,allagents2,pointset14,pointset21);
total_cost_fn=0;
%defining alpha & gain value,which varies with the deployment objective
% alpha=-1;gain=0.01; % for minimum variance controller
% alpha=1;gain=0.0001; for achieving consen
alpha=-10;gain=0.001
 % Initial plotting of  all the  agent positions
for n1=1:length(allagents1)
    axis([1 3 1 3])
    plot(allagents1(n1,1),allagents1(n1,2), 'ob', 'MarkerSize', 10)
    hold on
end

% updating the agent position in each iteration with respect to control action
for t=1:310
      if (t==1)
            for n_init=1:length(allagents)
                a=((n_init*2)-1);
                b=(n_init*2);
                A(a,t)=allagents(n_init,1);
                A(b,t)=allagents(n_init,2);
           end
           val=path_ploting(A,allagents);
     end
     title('Unifying controller without obstacles:Consensus')
%     implementing dp=k(dh/dp)
    for n=1:length(allagents)
        for n1=1:length(allagents1)
            plot(allagents1(n1,1),allagents1(n1,2), 'ob', 'MarkerSize', 10)
        end
        con_out=gradient_control(allagents(n,:),allagents,alpha);
        initial_agent_val(n,1)=allagents(n,1);
        initial_agent_val(n,2)=allagents(n,2);
        allagents(n,1)=allagents(n,1)-gain*con_out(1);
        allagents(n,2)=allagents(n,2)-gain*con_out(2);
    %creating path of agents as a matrix
        a=((n*2)-1);
        b=(n*2);
        a1=t+1;
        b1=t+1;
        A(a,a1)=allagents(n,1);
        A(b,b1)=allagents(n,2);   
        axis([1 3 1 3])
        title('Unifying controller')
        subplot(1,1,1)
        plot(allagents(n,1),allagents(n,2), '--o', 'MarkerSize', 8, 'MarkerFaceColor', 'b')   
        hold on
        subplot(1,1,1)
        axis([1 3 1 3])
        hold on
   end
   voronoi(allagents(:,1),allagents(:,2))
   val=path_ploting(A,allagents);
   hold off
end   
 %Plotting path of agents as a matrix 
function val=path_ploting(X,allagents) 
   for n2=1:length(allagents)
       c=((n2*2)-1);
       c1=(n2*2);  
       axis([1 3 1 3])
       plot(X(c,:),X(c1,:),':b','MarkerSize', 20)
       axis([1 3 1 3])
       hold on
       val=0;
     pause(0.00000001)
   end   
 end
        
%sensing cost and its derivative calculation
function [single_cost_fn,df_sensing_cost]=sensing_fn(agent,q)
    d =[q(1),q(2)]- [agent(1),agent(2)];
    single_cost_fn = (norm(d)^2)/2;
    df_sensing_cost=-([q(1),q(2)]- [agent(1),agent(2)]);              
end
%Mixing function
function mixing_cost=mixing_fn(allagents,q,alpha)
    total_cost_fn = 0;
    for n=1:length(allagents)
        single_cost_fn=sensing_fn(allagents(n,:),q);
        total_cost_fn=total_cost_fn+(single_cost_fn^alpha);
    end
    mixing_cost=(total_cost_fn)^(1/alpha);    
end
%Importance function
function importance=Importance_fn(q,imp_q)
    importance=imp_q(q(1),q(2));
end
%Derivative of cost function for a location q
function s_gradient=grad(agent,q,allagents,alpha)
     
     [single_cost_fn,df_sensing_cost]=sensing_fn(agent,q);
     mixing_cost=mixing_fn(allagents,q,alpha);
     importance=.1;
     s_gradient=(((single_cost_fn/ (mixing_cost+0.0000001))^(alpha-1))*df_sensing_cost*importance);
     if (isnan(s_gradient(1,1)) || isnan(s_gradient(1,2)))
        s_gradient=[0 0]     
     end
end
%Derivative of cost function for all points in  region Q
function s_t_gradient=gradient_control(agent,allagents,alpha)
    s_t_gradient = 0;
    for i=1:.01:3
        for j=1:.01:3
        q=[i,j];
        s_gradient=grad(agent,q,allagents,alpha);
        s_t_gradient=s_t_gradient+s_gradient;
        end  
    end      
end
    
