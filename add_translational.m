function mbs = add_translational(mbs, name, body1_name, s1, s2, body2_name, s3)
%  Add_translational Function that adds prismatic joints.
% translational joint definition: name, body names, local vector coordinates.
p = inputParser;
is_2vec = @(x) isvector(x) && length(x) == 2;
addRequired(p,'name', @isstring);
addRequired(p, 'body1_name', @isstring);
addRequired(p, 's1', is_2vec);               
 
addRequired(p, 's2', is_2vec);               
addRequired(p, 'body2_name', @isstring);
addRequired(p, 's3', is_2vec);              
parse(p, name, body1_name, s1, s2, body2_name, s3);

% Get bodies ids
b1 = get_body_id(mbs, body1_name);
b2 = get_body_id(mbs, body2_name);
tran = struct("name", name, "body1", b1, "s1", s1(:), "s2", s2(:), "body2", b2, "s3", s3(:));
mbs.joints.translational = [mbs.joints.translational, tran];
mbs.nc = mbs.nc + 2;
end