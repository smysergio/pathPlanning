
function xdot = model(x,u)
				
% [p, r, beta, phi, psi]

%xdot update matrix from current state
A = [ 0 0 1 0;
      0 0 0 1;
      0 0 0 0;
      0 0 0 0; ];

%xdot update matrix from inputs
B = [ 0 0;
      0 0;
      1 0;
      0 1;
];
xdot = A*x +B*u;

end

