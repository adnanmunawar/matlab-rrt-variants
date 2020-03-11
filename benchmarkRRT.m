% ==============================================================================
%
%    Software License Agreement (BSD License)
%    Copyright (c) 2019
%    (www.aimlab.wpi.edu)
%
%    All rights reserved.
%
%    Redistribution and use in source and binary forms, with or without
%    modification, are permitted provided that the following conditions
%    are met:
%
%    * Redistributions of source code must retain the above copyright
%    notice, this list of conditions and the following disclaimer.
%
%    * Redistributions in binary form must reproduce the above
%    copyright notice, this list of conditions and the following
%    disclaimer in the documentation and/or other materials provided
%    with the distribution.
%
%    * Neither the name of authors nor the names of its contributors may
%    be used to endorse or promote products derived from this software
%    without specific prior written permission.
%
%    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
%    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
%    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
%    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
%    COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
%    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
%    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
%    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
%    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
%    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
%    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
%    POSSIBILITY OF SUCH DAMAGE.
%
%    \author:    <http://www.aimlab.wpi.edu>
%    \author:    <amunawar@wpi.edu>
%    \author:    Adnan Munawar
%    \version:   0.1$
% ==============================================================================

function benchmarkRRT

clc;
close all;
clear all;


num_of_runs =1;
run_RRTconnect =0;
run_RRTextend = 0;
run_LazyRRT = 0;
run_RRTstar = 1;

dim = 2;
stepsize = [5];

random_world = 0;
radius = 10;
samples = 4000;

show_output = 1;
show_benchmark_results = 0;


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



t_lazy = [];
t_extend = [];
t_connect = [];
t_star = [];

l_lazy = [];
l_extend = [];
l_connect = [];
l_star = [];

p_lazy = [];
p_extend = [];
p_connect = [];
p_star = [];

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for sits = 1:size(stepsize,2)
    segmentLength = stepsize(sits);
    if run_LazyRRT == 1
        time = 0;
        avg_its = 0;
        avg_path = 0;
        
        for i = 1:num_of_runs
            
            [n_its path_n run_time] =  LazyRRT3D(dim,segmentLength,random_world,show_output);
            time = time + run_time;
            avg_its = avg_its + n_its;
            avg_path = avg_path + path_n;
        end
        
        str1 = ['The time taken by Lazy RRT for ', num2str(num_of_runs), ' runs is ', num2str(time)];
        str2 = ['The averagae time taken by Lazy RRT for each run is ', num2str(time/num_of_runs)];
        str3 = ['The averagae number of states explored by Lazy RRT for each run is ', num2str(avg_its/num_of_runs)];
        str4 = ['The averagae number of state in Path by Lazy RRT for each run is ', num2str(avg_path/num_of_runs)];
        
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
        disp(str1);
        disp(str2);
        disp(str3);
        disp(str4);
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
        
        t_lazy = [t_lazy time/num_of_runs];
        l_lazy = [l_lazy avg_its/num_of_runs];
        p_lazy = [p_lazy avg_path/num_of_runs];
        
    end
    
    
    
    if run_RRTstar == 1
        time = 0;
        avg_its = 0;
        avg_path = 0;
        for i = 1:num_of_runs
            [n_its path_n,run_time] = RRTstar3D(dim,segmentLength,radius,random_world,show_output,samples);
            time = time + run_time;
            avg_its = avg_its + n_its;
            avg_path = avg_path + path_n;
        end
        
        str1 = ['The time taken by RRT-Star for ', num2str(num_of_runs), ' runs is ', num2str(time)];
        str2 = ['The averagae time taken by RRT_Star for each run is ', num2str(time/num_of_runs)];
        str3 = ['The averagae number of states explored by RRT_Star for each run is ', num2str(avg_its/num_of_runs)];
        str4 = ['The averagae number of state in Path by RRT-Star for each run is ', num2str(avg_path/num_of_runs)];
        
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
        disp(str1);
        disp(str2);
        disp(str3);
        disp(str4);
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
        
        t_star = [t_star time/num_of_runs];
        l_star = [l_star avg_its/num_of_runs];
        p_star = [p_star avg_path/num_of_runs];
        
    end
    
    if run_RRTextend == 1
        time = 0;
        avg_its = 0;
        avg_path = 0;
        for i = 1:num_of_runs
            [n_its path_n,run_time] = RRTextend3D(dim,segmentLength,random_world,show_output);
            time = time + run_time;
            avg_its = avg_its + n_its;
            avg_path = avg_path + path_n;
        end
        
        str1 = ['The time taken by RRT-Extend for ', num2str(num_of_runs), ' runs is ', num2str(time)];
        str2 = ['The averagae time taken by RRT_Extend for each run is ', num2str(time/num_of_runs)];
        str3 = ['The averagae number of states explored by RRT_Extend for each run is ', num2str(avg_its/num_of_runs)];
        str4 = ['The averagae number of state in Path by RRT-Extend for each run is ', num2str(avg_path/num_of_runs)];
        
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
        disp(str1);
        disp(str2);
        disp(str3);
        disp(str4);
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
        
        t_extend = [t_extend time/num_of_runs];
        l_extend = [l_extend avg_its/num_of_runs];
        p_extend = [p_extend avg_path/num_of_runs];
        
    end
    
    if run_RRTconnect == 1
        
        time = 0;
        avg_its = 0;
        avg_path = 0;
        
        for i = 1:num_of_runs
            [n_its path_n,run_time] =  RRTconnect3D(dim,segmentLength,random_world,show_output);
            time = time + run_time;
            avg_its = avg_its + n_its;
            avg_path = avg_path + path_n;
        end
        
        str1 = ['The time taken by RRT-Connect for ', num2str(num_of_runs), ' runs is ', num2str(time)];
        str2 = ['The averagae time taken by RRT-Connect for each run is ', num2str(time/num_of_runs)];
        str3 = ['The averagae number of states explored by RRT-Connect for each run is ', num2str(avg_its/num_of_runs)];
        str4 = ['The averagae number of state in Path by RRT-Connect for each run is ', num2str(avg_path/num_of_runs)];
        
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
        disp(str1);
        disp(str2);
        disp(str3);
        disp(str4);
        disp('%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%');
        
        t_connect = [t_connect time/num_of_runs];
        l_connect = [l_connect avg_its/num_of_runs];
        p_connect = [p_connect avg_path/num_of_runs];
    end
end

if show_benchmark_results == 1
    
    figure;
    hold on;
    plot(stepsize,t_lazy,'r','LineWidth',2);
    plot(stepsize,t_extend,'g','LineWidth',2);
    plot(stepsize,t_connect,'b','LineWidth',2);
    ylabel('Computational Time');
    xlabel('Step Size');
    dim_str = sprintf('Comparison of computational time for %d Dimensional C-Space',dim);
    title(dim_str)
    hold off;
    
    figure;
    hold on;
    plot(stepsize,l_lazy,'r','LineWidth',2);
    plot(stepsize,l_extend,'g','LineWidth',2);
    plot(stepsize,l_connect,'b','LineWidth',2);
    ylabel('Number of States Explored');
    xlabel('Step Size');
    dim_str = sprintf(' Comparison of number of states explored for %d Dimensional C-Space',dim);
    title(dim_str)
    hold off;
    
    figure;
    hold on;
    plot(stepsize,p_lazy,'r','LineWidth',2);
    plot(stepsize,p_extend,'g','LineWidth',2);
    plot(stepsize,p_connect,'b','LineWidth',2);
    ylabel('Number of States in Path');
    xlabel('Step Size');
    dim_str = sprintf('Comparison for number of states in path for %d Dimensional C-Space',dim);
    title(dim_str)
    hold off;
    
end




%     t_lazy
%     l_lazy
%     p_lazy
%
%
%
%     t_extend
%     l_extend
%     p_extend
%
%     t_connect
%     l_connect
%     p_connect

end
