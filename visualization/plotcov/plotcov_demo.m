% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Generate some sample data for presentation.
% Note that plotcov itself only needs the mean and covariance matrix.

num = 10;

% Draw 50 samples from class 1 with mean `m1` and covariance `C1`.
X1 = mvnrnd([.5 1.5], [.1 .1], num);
m1 = mean(X1);
C1 = cov(X1);

% Draw 50 samples from class 2 with mean `m2` and covariance `C2`.
X2 = mvnrnd([1 1], [.2 .1], num);
m2 = mean(X2);
C2 = cov(X2);

X  = [X1; X2];
g  = [zeros(num,1); ones(num,1)];

gscatter(X(:,1), X(:,2), g);
axis equal;

% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Use plotcov to draw ellipses at 1, 2 and 3 standard deviations.

hold on;
plotcov(C1, m1);        % draw ellipses for class 1
plotcov(C2, m2);        % draw ellipses for class 2
hold off;
