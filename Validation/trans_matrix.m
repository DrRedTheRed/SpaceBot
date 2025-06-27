function trans_matrix = trans(afa,a,d,theta)
      len = length(afa);
      syms T [4 4 len]
      for i = 1:len
          T(:,:,i) = trans_cal( afa(i),a(i),d(i),theta(i) );
          if(i == 1)
              trans_matrix = T(:,:,i);
          else
              trans_matrix = trans_matrix*T(:,:,i);
          end  
      end
end