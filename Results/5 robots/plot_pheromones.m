filename = 'pheromone_history_10.csv';

data = readmatrix(filename);

n_patrols = size(data, 2);
NUM_ITER  = (size(data,1)-2)/n_patrols;


% Extract patrol positions
patrol_x = data(1, 1:n_patrols);
patrol_y = -data(2, 1:n_patrols);
patrol_y_inv=-patrol_y;

init_table = ones(n_patrols, n_patrols)*0.001;

start_row = 3;
rows_per_iter = n_patrols;
pher_history = zeros(n_patrols, n_patrols, NUM_ITER);
%pher_history(:,:,1) = init_table;

% Extract pheromone tables
for iter = 1:NUM_ITER
    r0 = start_row + (iter-1)*rows_per_iter;
    r1 = r0 + rows_per_iter - 1;
    block = data(r0:r1, 1:n_patrols);
    pher_history(:,:,iter) = block;
end
%%

x = linspace(-2,2,1001);
y = linspace(-2,2,1001);
tol   = 0.04; % thickness in world units

for iter = 1:NUM_ITER
    paths = zeros(length(y), length(x));   % rows = y, cols = x
    
    table = pher_history(:,:,iter);   % n_patrols x n_patrols
    
    for pat1 = 1:n_patrols-1
        for pat2 = pat1+1:n_patrols
            x1 = patrol_x(pat1);
            y1 = patrol_y(pat1);
            x2 = patrol_x(pat2);
            y2 = patrol_y(pat2);
    
            dx = x2 - x1;
            dy = y2 - y1;
            segLen2 = dx*dx + dy*dy;
    
            for ix = 1:length(x)
                for iy = 1:length(y)
                    x0 = x(ix);
                    y0 = y(iy);
    
                    % projection of (x0,y0) onto the infinite line, parametrized by t
                    t = ((x0 - x1)*dx + (y0 - y1)*dy) / segLen2;
    
                    % only keep the part of the infinite line that lies on the segment
                    if t < 0 || t > 1
                        continue;
                    end
    
                    % closest point on the segment
                    x_proj = x1 + t*dx;
                    y_proj = y1 + t*dy;
    
                    % Euclidean distance from point to segment
                    dist = sqrt((x0 - x_proj)^2 + (y0 - y_proj)^2);
    
                    if dist < tol
                        % use max to avoid overwriting a stronger edge with a weaker one
                        paths(iy, ix) = max(paths(iy, ix), table(pat1, pat2));
                    end
                end
            end
        end
    end
    
    
    paths_display = paths;
    paths_display(paths_display == 0) = NaN;
    
    h = heatmap(x, y, paths_display);
    h.ColorLimits = [min(pher_history(:)), max(pher_history(:))];
    h.MissingDataColor = [1 1 1];  % white
    h.MissingDataLabel = 'No pheromone';
    colormap("parula");
    colorbar;
    title('Pheromone heat map')
    grid off
    % Number of x and y categories
    nx = numel(h.XDisplayData);
    ny = numel(h.YDisplayData);
    
    % Hide x and y axis tick labels
    h.XDisplayLabels = repmat({''}, 1, nx);
    h.YDisplayLabels = repmat({''}, ny, 1);
    pause(0);
end
%%
it_nb = 4;
table = pher_history(:,:,it_nb) - pher_history(1,1,it_nb)*eye(n_patrols);
G = graph(table);
weights = G.Edges.Weight;
figure;
plot(G, 'EdgeCData', weights, 'LineWidth',4, ...
    'XData',patrol_x, 'YData',patrol_y_inv, 'EdgeAlpha',1)
colormap('jet')
colorbar
%%

% Build your original graph
table = pher_history(:,:,it_nb) - pher_history(1,1,it_nb)*eye(n_patrols);
G = graph(table);

% Get weights and edge list
E = G.Edges;
weights = E.Weight;

% Sort edges by weight (smallest first, largest last)
[~, idx] = sort(weights, 'ascend');

% Rebuild graph with edges in sorted order
G2 = graph(E.EndNodes(idx,1), E.EndNodes(idx,2), weights(idx), numnodes(G));

% Plot – high-weight edges get drawn last, so they appear on top
figure;
p = plot(G2, ...
    'EdgeCData', G2.Edges.Weight, ...
    'LineWidth', 4, ...
    'XData', patrol_x, ...
    'YData', patrol_y_inv, ...
    'EdgeAlpha', 1);

colormap('jet');
colorbar;

