function [mu_lr, mu_blr, cov_blr] = param_id(id_data)
%%Linear Regression 
X = [];
D = size(id_data.input_cur,2);
for i = 1:D
    ak = id_data.input_cur(i);
    pk = id_data.state_cur(1,i);
    vk = id_data.state_cur(2,i);
    y  = id_data.state_nxt;
    mat_x = [ak,-cos(3*pk);ak,-cos(3*pk)];
    X = [X;mat_x];
    Y(2*i-1,1) = y(1,i) - pk - vk;
    Y(2*i,1) = y(2,i) - vk;
end
mu_lr = inv(X'*X) * X'*Y;

%%Bayesian Linear Regression
sig = 0.00015;
mu_0 = 0;
cov_0 = diag([sig^2,sig^2]);
cov_hat_inv = inv(cov_0) + sig^(-2) * X'*X;
mu_hat = inv(cov_hat_inv)*(inv(cov_0)*mu_0 + sig^(-2)*X'*Y);
mu_blr = mu_hat(:,1);
cov_blr = inv(cov_hat_inv);

end

