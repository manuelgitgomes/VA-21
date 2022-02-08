function z = measurementFcn(x, varargin)
    cartState = filterToCartState(x);
    z = cvmeas(cartState,varargin{:});
    if size(z,1) == 2
        z(3,1) = -0.1;
    end
end