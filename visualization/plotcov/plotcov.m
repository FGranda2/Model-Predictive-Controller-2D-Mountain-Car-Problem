function [hf, hs] = plotcov(C, mu, varargin)
% Visualise a 2x2 covariance matrix by drawing ellipses at 1, 2 and 3 STD.
%
% Input arguments:
%  C         2x2 Covariance matrix.
%  MU        Optional 1x2 array defining the centre of the ellipses.
%            The default value is [0,0].
%  VARARGIN  Any keyword arguments can be passed to `plot`.
%
% Output arguments:
%  H         3x1 vector of plot handles. One per ellipse. In order, they
%            are the handles to the ellipses at 1, 2 and 3 STD.
%
  if nargin<2 || isempty(mu), mu=[0 0]; end
  
  % Find sorted eigenvectors and eigenvalues for C.
  [V,D]  = eig(C);
  [~,ix] = sort(diag(D), 'descend');
  D      = D(ix,ix);
  V      = V(:,ix);
  
  % Define the scales at which to draw the ellipses.
  stds   = 3; %[1 2 3]; % 1, 2 and 3 standard deviations.
  conf   = 2 * normcdf(stds) - 1;
  scale  = chi2inv(conf, 2);
  
  % Set up a circle. Will be scaled to ellipses.
  t = linspace(0, 2*pi, 100)';
  e = [cos(t) sin(t)];
  
  % Line styles (and handles) for the ellipses.
  styles = {
    {'LineWidth',2};    % 1 STD
    {};                 % 2 STD
    {'LineStyle',':'}   % 3 STD
  };
  h = zeros(numel(stds), 1);
  
  washold = ishold;
  
  for i = 1:numel(stds)
    % Set up the i-th scaled ellipse.
    VV    = V * sqrt(D * scale(i));
    ee    = bsxfun(@plus, e*(VV'), mu);
    
    % Draw with different properties for different ellipses.
    % Also, pick the colour from the first ellipse for aesthetics.
    if i == 1
      hf = fill(ee(:,1), ee(:,2), '', varargin{:});
      c    = get(hf, 'FaceColor');
      a    = get(hf, 'FaceAlpha');
      hold on;
    else
      hf = fill(ee(:,1), ee(:,2), '', varargin{:});
    end
  end
  hs = scatter(mu(1), mu(2), 'o', 'MarkerEdgeColor', 'None',...
      'MarkerEdgeAlpha', a, 'MarkerFaceColor', c, ...
      'MarkerFaceAlpha', a);
  
  if ~washold
    hold off;
  end
end
