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

function [its,sizePath,run_time] =  RRTstar3D(dim,segmentLength,radius,random_world,show_output,samples)
% if samples < 4000
%     disp('ERROR! SPECIFY ATLEAST 4000 SAMPLES')
%     return
% end

% dim = 2;
% radius =0;
% segmentLength = 5;
% random_world = 0;
% n_its = 1000;
% standard length of path segments
if dim ==2
    start_cord = [5,5];
    goal_cord = [95,95];
    
else
    
    start_cord = [5,5,5];
    goal_cord = [95,95,95];
end



% create random world
Size = 100;
NumObstacles = 100;

if random_world ==1
    world = createWorld(NumObstacles,ones(1,dim)*Size,zeros(1,dim),dim);
else
    [world NumObstacles] = createKnownWorld(ones(1,dim)*Size,[0;0;0],dim);
end
% randomly select start and end nodes
%start_node = generateRandomNode(world,dim)
%end_node   = generateRandomNode(world,dim)
start_node = [start_cord,0,0,0];
end_node = [goal_cord,0,0,0];
% establish tree starting with the start node
tree = start_node;

numPaths = 0;
a = clock;
% check to see if start_node connects directly to end_node
if ( (norm(start_node(1:dim)-end_node(1:dim))<segmentLength )...
        &&(collision(start_node,end_node,world,dim)==0) )
    
    disp('CASE 1')
    path = [start_node; end_node];
else
    disp('CASE 2')
    if samples >0
        draw = floor(samples/8);
        its = 0;
        for i = 1:samples
            flag = 0;
            [tree,flag] = extendTree(tree,end_node,segmentLength,radius,world,flag,dim);
            numPaths = numPaths + flag;
            its = its+1;
            
            if its == draw
                tree_1 = tree;
            elseif its == draw*2
                tree_2 = tree;
            elseif its == draw*3
                tree_3 = tree;
            elseif its == draw*4
                tree_4 = tree;
            elseif its == draw*5
                tree_5 = tree;
            elseif its == draw*6
                tree_6 = tree;
            elseif its == draw*7
                tree_7 = tree;
            elseif its == samples
                tree_8 = tree;
            end
        end
        
    else
        its = 0;
        numPaths = 0;
        flag = 0;
        while numPaths < 1,
            [tree,flag] = extendTree(tree,end_node,segmentLength,radius,world,flag,dim);
            numPaths = numPaths + flag;
            its = its+1;
        end
    end
    
end
numPaths

% find path with minimum cost to end_node
path = findMinimumPath(tree,end_node,dim);

b = clock;
run_time = 3600*(b(4)-a(4)) + 60 * (b(5)-a(5)) + (b(6) - a(6));

path_1 = findMinimumPath(tree_1,end_node,dim);

path_2 = findMinimumPath(tree_2,end_node,dim);

path_3 = findMinimumPath(tree_3,end_node,dim);

path_4 = findMinimumPath(tree_4,end_node,dim);

path_5 = findMinimumPath(tree_5,end_node,dim);

path_6 = findMinimumPath(tree_6,end_node,dim);

path_7 = findMinimumPath(tree_7,end_node,dim);

path_8 = findMinimumPath(tree_8,end_node,dim);

sizePath = size(path,1);


if show_output == 1
    
    if size(path_1, 1) > 0
        figure;
        plotExpandedTree(world,tree_1,dim);
        plotWorld(world,path_1,dim);
    else
        disp('COULD NOT FIND A CONNECTING TREE TILL 1/8th SAMPLES SO NOT DRAWING THAT PATH')
    end
    if size(path_2, 1) > 0
        figure;
        plotExpandedTree(world,tree_2,dim);
        plotWorld(world,path_2,dim);
    else
        disp('COULD NOT FIND A CONNECTING TREE TILL 2/8th SAMPLES SO NOT DRAWING THAT PATH')
    end
    if size(path_3, 1) > 0
    figure;
    plotExpandedTree(world,tree_3,dim);
    plotWorld(world,path_3,dim);
    else
        disp('COULD NOT FIND A CONNECTING TREE TILL 3/8th SAMPLES SO NOT DRAWING THAT PATH')
    end
    if size(path_4, 1) > 0
    figure;
    plotExpandedTree(world,tree_4,dim);
    plotWorld(world,path_4,dim);
    else
        disp('COULD NOT FIND A CONNECTING TREE TILL 4/8th SAMPLES SO NOT DRAWING THAT PATH')
    end
    if size(path_5, 1) > 0
    figure;
    plotExpandedTree(world,tree_5,dim);
    plotWorld(world,path_5,dim);
    else
        disp('COULD NOT FIND A CONNECTING TREE TILL 5/8th SAMPLES SO NOT DRAWING THAT PATH')
    end
    if size(path_6, 1) > 0
    figure;
    plotExpandedTree(world,tree_6,dim);
    plotWorld(world,path_6,dim);
    else
        disp('COULD NOT FIND A CONNECTING TREE TILL 6/8th SAMPLES SO NOT DRAWING THAT PATH')
    end
    if size(path_7, 1) > 0
    figure;
    plotExpandedTree(world,tree_7,dim);
    plotWorld(world,path_7,dim);
    else
        disp('COULD NOT FIND A CONNECTING TREE TILL 7/8th SAMPLES SO NOT DRAWING THAT PATH')
    end
    if size(path_8, 1) > 0
    figure;
    plotExpandedTree(world,tree_8,dim);
    plotWorld(world,path_8,dim);
    else
        disp('COULD NOT FIND A CONNECTING TREE TILL 8/8th SAMPLES SO NOT DRAWING THAT PATH')
    end
    if size(path, 1) > 0
    figure;
    plotExpandedTree(world,tree,dim);
    plotWorld(world,path,dim);
    else
        disp('COULD NOT FIND A CONNECTING TREE FOR THE SPECIFIED SAMPLES. PLEASE INCREASE THE NUMBER OF SAMPLES')
    end
end
end





function world = createWorld(NumObstacles, endcorner, origincorner,dim)

if dim == 2
    
    % check to make sure that the region is nonempty
    if (endcorner(1) <= origincorner(1)) | (endcorner(2) <= origincorner(2))
        disp('Not valid corner specifications!')
        world=[];
        
        % create world data structure
    else
        world.NumObstacles = NumObstacles;
        world.endcorner = endcorner;
        world.origincorner = origincorner;
        
        % create NumObstacles
        maxRadius = min(endcorner(1)- origincorner(1), endcorner(2)-origincorner(2));
        maxRadius = 5*maxRadius/NumObstacles/2;
        for i=1:NumObstacles,
            % randomly pick radius
            world.radius(i) = maxRadius*rand;
            % randomly pick center of obstacles
            cx = origincorner(1) + world.radius(i)...
                + (endcorner(1)-origincorner(1)-2*world.radius(i))*rand;
            cy = origincorner(2) + world.radius(i)...
                + (endcorner(2)-origincorner(2)-2*world.radius(i))*rand;
            world.cx(i) = cx;
            world.cy(i) = cy;
        end
    end
    
elseif dim ==3;
    % check to make sure that the region is nonempty
    if (endcorner(1) <= origincorner(1)) || (endcorner(2) <= origincorner(2)) || (endcorner(3) <= origincorner(3))
        disp('Not valid corner specifications!')
        world=[];
        
        % create world data structure
    else
        world.NumObstacles = NumObstacles;
        world.endcorner = endcorner;
        world.origincorner = origincorner;
        
        % create NumObstacles
        bounds = [endcorner(1)- origincorner(1), endcorner(2)-origincorner(2), endcorner(3)-origincorner(3)];
        maxRadius = min(bounds);
        maxRadius = 5*maxRadius/NumObstacles;
        for i=1:NumObstacles,
            % randomly pick radius
            world.radius(i) = maxRadius*rand;
            % randomly pick center of obstacles
            cx = origincorner(1) + world.radius(i)...
                + (endcorner(1)-origincorner(1)-2*world.radius(i))*rand;
            cy = origincorner(2) + world.radius(i)...
                + (endcorner(2)-origincorner(2)-2*world.radius(i))*rand;
            cz = origincorner(2) + world.radius(i)...
                + (endcorner(2)-origincorner(2)-2*world.radius(i))*rand;
            world.cx(i) = cx;
            world.cy(i) = cy;
            world.cz(i) = cz;
        end
    end
end
end

function [world NumObstacles] = createKnownWorld(endcorner, origincorner,dim)
NumObstacles = 5;
if dim == 2
    % check to make sure that the region is nonempty
    if (endcorner(1) <= origincorner(1)) | (endcorner(2) <= origincorner(2)),
        disp('Not valid corner specifications!')
        world=[];
        % create world data structure
    else
        world.NumObstacles = NumObstacles;
        world.endcorner = endcorner;
        world.origincorner = origincorner;
        
        % create NumObstacles
        maxRadius = 10;
        
        world.radius(1) = maxRadius;
        cx = 50;
        cy = 50;
        world.cx(1) = cx;
        world.cy(1) = cy;
        
        world.radius(2) = maxRadius;
        cx = 75;
        cy = 25;
        world.cx(2) = cx;
        world.cy(2) = cy;
        
        world.radius(3) = maxRadius;
        cx = 25;
        cy = 75;
        world.cx(3) = cx;
        world.cy(3) = cy;
        
        world.radius(4) = maxRadius;
        cx = 25;
        cy = 25;
        world.cx(4) = cx;
        world.cy(4) = cy;
        
        world.radius(5) = maxRadius;
        cx = 75;
        cy = 75;
        world.cx(5) = cx;
        world.cy(5) = cy;
    end
    
elseif dim == 3
    
    NumObstacles = 9;
    % check to make sure that the region is nonempty
    if (endcorner(1) <= origincorner(1)) | (endcorner(2) <= origincorner(2)) | (endcorner(3) <= origincorner(3)),
        disp('Not valid corner specifications!')
        world=[];
        
        % create world data structure
    else
        world.NumObstacles = NumObstacles;
        world.endcorner = endcorner;
        world.origincorner = origincorner;
        
        % create NumObstacles
        maxRadius = 10;
        
        world.radius(1) = maxRadius;
        cx = 50;
        cy = 50;
        cz = 50;
        world.cx(1) = cx;
        world.cy(1) = cy;
        world.cz(1) = cz;
        
        world.radius(2) = maxRadius;
        cx = 25;
        cy = 25;
        cz = 25;
        world.cx(2) = cx;
        world.cy(2) = cy;
        world.cz(2) = cz;
        
        world.radius(3) = maxRadius;
        cx = 75;
        cy = 75;
        cz = 75;
        world.cx(3) = cx;
        world.cy(3) = cy;
        world.cz(3) = cz;
        
        world.radius(4) = maxRadius;
        cx = 25;
        cy = 25;
        cz = 75;
        world.cx(4) = cx;
        world.cy(4) = cy;
        world.cz(4) = cz;
        
        world.radius(5) = maxRadius;
        cx = 75;
        cy = 75;
        cz = 25;
        world.cx(5) = cx;
        world.cy(5) = cy;
        world.cz(5) = cz;
        
        world.radius(6) = maxRadius;
        cx = 25;
        cy = 75;
        cz = 25;
        world.cx(6) = cx;
        world.cy(6) = cy;
        world.cz(6) = cz;
        
        world.radius(7) = maxRadius;
        cx = 75;
        cy = 25;
        cz = 25;
        world.cx(7) = cx;
        world.cy(7) = cy;
        world.cz(7) = cz;
        
        world.radius(8) = maxRadius;
        cx = 75;
        cy = 25;
        cz = 75;
        world.cx(8) = cx;
        world.cy(8) = cy;
        world.cz(8) = cz;
        
        
        world.radius(9) = maxRadius;
        cx = 25;
        cy = 75;
        cz = 75;
        world.cx(9) = cx;
        world.cy(9) = cy;
        world.cz(9) = cz;
    end
end
end





function node=generateRandomNode(world,dim)

if dim ==2;
    % randomly pick configuration
    px       = (world.endcorner(1)-world.origincorner(1))*rand;
    py       = (world.endcorner(2)-world.origincorner(2))*rand;
    
    chi      = 0;
    cost     = 0;
    node     = [px, py, chi, cost, 0];
    
    % check collision with obstacle
    while collision(node, node, world,dim),
        px       = (world.endcorner(1)-world.origincorner(1))*rand;
        py       = (world.endcorner(2)-world.origincorner(2))*rand;
        
        chi      = 0;
        cost     = 0;
        node     = [px, py, chi, cost, 0];
    end
    
elseif dim ==3;
    % randomly pick configuration
    px       = (world.endcorner(1)-world.origincorner(1))*rand;
    py       = (world.endcorner(2)-world.origincorner(2))*rand;
    pz       = (world.endcorner(3)-world.origincorner(3))*rand;
    
    chi      = 0;
    cost     = 0;
    node     = [px, py, pz, chi, cost, 0];
    
    % check collision with obstacle
    while collision(node, node, world,dim),
        px       = (world.endcorner(1)-world.origincorner(1))*rand;
        py       = (world.endcorner(2)-world.origincorner(2))*rand;
        pz       = (world.endcorner(3)-world.origincorner(3))*rand;
        
        chi      = 0;
        cost     = 0;
        node     = [px, py, pz, chi, cost, 0];
    end
    
end

end





function collision_flag = collision(node, parent, world,dim)

collision_flag = 0;


for i=1:dim
    if (node(i)>world.endcorner(i))|(node(i)<world.origincorner(i))
        collision_flag = 1;
    end
end

if collision_flag == 0 && dim ==2
    for sigma = 0:.2:1,
        p = sigma*node(1:dim) + (1-sigma)*parent(1:dim);
        % check each obstacle
        for i=1:world.NumObstacles,
            if (norm([p(1);p(2)]-[world.cx(i); world.cy(i)])<=1*world.radius(i)),
                collision_flag = 1;
                break;
            end
        end
    end
    
elseif collision_flag == 0 && dim ==3
    for sigma = 0:.2:1,
        p = sigma*node(1:dim) + (1-sigma)*parent(1:dim);
        % check each obstacle
        for i=1:world.NumObstacles,
            if (norm([p(1);p(2);p(3)]-[world.cx(i); world.cy(i); world.cz(i)])<=1*world.radius(i)),
                collision_flag = 1;
                break;
            end
        end
    end
end
end







function flag = canEndConnectToTree(tree,end_node,minDist,world,dim)
flag = 0;
% check only last node added to tree since others have been checked
if ( (norm(tree(end,1:dim)-end_node(1:dim))<minDist)...
        & (collision(tree(end,1:dim), end_node(1:dim), world,dim)==0) ),
    flag = 1;
end

end








function [new_tree,flag] = extendTree(tree,end_node,segmentLength,r,world,flag_chk,dim)

flag1 = 0;
while flag1==0,
    % select a random point
    randomPoint = ones(1,dim);
    for i=1:dim
        randomPoint(1,i) = (world.endcorner(i)-world.origincorner(i))*rand;
    end
    
    % find leaf on node that is closest to randomPoint
    tmp = tree(:,1:dim)-ones(size(tree,1),1)*randomPoint;
    sqrd_dist = sqr_eucl_dist(tmp,dim);
    [min_dist,idx] = min(sqrd_dist);
    min_parent_idx = idx;
    
    new_point = (randomPoint-tree(idx,1:dim));
    new_point = tree(idx,1:dim)+(new_point/norm(new_point))*segmentLength;
    
    min_cost  = cost_np(tree(idx,:),new_point,dim);
    new_node  = [new_point, 0, min_cost, idx];
    
    if collision(new_node, tree(idx,:), world,dim)==0
        
        tmp_dist = tree(:,1:dim)-(ones(size(tree,1),1)*new_point);
        dist = sqr_eucl_dist(tmp_dist,dim);
        near_idx = find(dist <= r^2);
        
        if size(near_idx,1)>1
            size_near = size(near_idx,1);
            
            for i = 1:size_near
                if collision(new_node, tree(near_idx(i),:), world,dim)==0
                    
                    cost_near = tree(near_idx(i),dim+2)+line_cost(tree(near_idx(i),:),new_point,dim);
                    
                    if  cost_near < min_cost
                        min_cost = cost_near;
                        min_parent_idx = near_idx(i);
                    end
                    
                end
            end
        end
        
        new_node = [new_point, 0 , min_cost, min_parent_idx];
        new_tree = [tree; new_node];
        new_node_idx = size(new_tree,1);
        
        if size(near_idx,1)>1
            reduced_idx = near_idx;
            for j = 1:size(reduced_idx,1)
                near_cost = new_tree(reduced_idx(j),dim+2);
                lcost = line_cost(new_tree(reduced_idx(j),:),new_point,dim);
                if near_cost > min_cost + lcost ...
                        && collision(new_tree(reduced_idx(j),:),new_node,world,dim)
                    before = new_tree(reduced_idx(j),dim+3)
                    new_tree(reduced_idx(j),dim+3) = new_node_idx;
                    after = new_tree(reduced_idx(j),dim+3)
                end
                
            end
        end
        flag1=1;
    end
end


if flag_chk == 0
    % check to see if new node connects directly to end_node
    if ( (norm(new_node(1:dim)-end_node(1:dim))<segmentLength )...
            && (collision(new_node,end_node,world,dim)==0) )
        flag = 1;
        new_tree(end,dim+1)=1;  % mark node as connecting to end.
    else
        flag = 0;
    end
    
else flag = 1;
end
end


function e_dist = sqr_eucl_dist(array,dim)

sqr_e_dist = zeros(size(array,1),dim);
for i=1:dim
    
    sqr_e_dist(:,i) = array(:,i).*array(:,i);
    
end
e_dist = zeros(size(array,1),1);
for i=1:dim
    
    e_dist = e_dist+sqr_e_dist(:,i);
    
end

end



%calculate the cost from a node to a point
function [cost] = cost_np(from_node,to_point,dim)

diff = from_node(:,1:dim) - to_point;
eucl_dist = norm(diff);
cost = from_node(:,dim+2) + eucl_dist;

end


%calculate the cost from a node to a node
function [cost] = cost_nn(from_node,to_node,dim)

diff = from_node(:,1:dim) - to_node(:,1:dim);
eucl_dist = norm(diff);
cost = from_node(:,dim+2) + eucl_dist;

end

function [cost] = line_cost(from_node,to_point,dim)
diff = from_node(:,1:dim) - to_point;
cost = norm(diff);
end


function path = findMinimumPath(tree,end_node,dim)

% find nodes that connect to end_node
connectingNodes = [];
for i=1:size(tree,1),
    if tree(i,dim+1)==1,
        connectingNodes = [connectingNodes ; tree(i,:)];
    end
end

if size(connectingNodes, 1) > 0
    
    % find minimum cost last node
    [tmp,idx] = min(connectingNodes(:,dim+2));
    
    % construct lowest cost path
    path = [connectingNodes(idx,:); end_node];
    parent_node = connectingNodes(idx,dim+3);
    while parent_node>1,
        parent_node = tree(parent_node,dim+3);
        path = [tree(parent_node,:); path];
    end
    
else
    path = [];
end

end


function plotExpandedTree(world,tree,dim)
ind = size(tree,1);
while ind>0
    branch = [];
    node = tree(ind,:);
    branch = [ branch ; node ];
    parent_node = node(dim+3);
    while parent_node > 1
        cur_parent = parent_node;
        branch = [branch; tree(parent_node,:)];
        parent_node = tree(parent_node,dim+3);
    end
    ind = ind - 1;
    
    if dim == 2
        X = branch(:,1);
        Y = branch(:,2);
        
        p = plot(X,Y);
        set(p,'Color','r','LineWidth',0.5,'Marker','.','MarkerEdgeColor','g');
        hold on;
        
    elseif dim == 3
        X = branch(:,1);
        Y = branch(:,2);
        Z = branch(:,3);
        
        p = plot3(X,Y,Z);
        set(p,'Color','r','LineWidth',0.5,'Marker','.','MarkerEdgeColor','g');
        hold on;
    end
end
end




function plotWorld(world,path,dim)
% the first element is the north coordinate
% the second element is the south coordinate
if dim ==2
    
    N = 10;
    th = 0:2*pi/N:2*pi;
    axis([world.origincorner(1),world.endcorner(1),...
        world.origincorner(2), world.endcorner(2)]);
    hold on
    
    for i=1:world.NumObstacles,
        X = world.radius(i)*sin(th) + world.cx(i);
        Y = world.radius(i)*cos(th) + world.cy(i);
        fill(X,Y,'blue');
    end
    
    X = path(:,1);
    Y = path(:,2);
    p = plot(X,Y);
    
elseif dim ==3
    axis([world.origincorner(1),world.endcorner(1),...
        world.origincorner(2), world.endcorner(2),...
        world.origincorner(3), world.endcorner(3)]);
    hold on
    
    for i=1:world.NumObstacles,
        [X Y Z] = sphere(10);
        X = (X*world.radius(i));
        Y = (Y*world.radius(i));
        Z = (Z*world.radius(i));
        surf(X+world.cx(i),Y+world.cy(i),Z+world.cz(i));
        colormap([0.5 0.2 0.3]);
    end
    
    X = path(:,1);
    Y = path(:,2);
    Z = path(:,3);
    p = plot3(X,Y,Z);
end
set(p,'Color','black','LineWidth',3)
xlabel('X axis');
ylabel('Y axis');
zlabel('Z axis');
title('RRT Star Algorithm');
end
