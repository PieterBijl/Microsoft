function q = dcm2quat_mod( dcm )
if any(~isreal(dcm) || ~isnumeric(dcm))
    error(message('aero:dcm2quat:isNotReal'));
end
if ((size(dcm,1) ~= 3) || (size(dcm,2) ~= 3))
    error(message('aero:dcm2quat:wrongDimension'));
end
for i = size(dcm,3):-1:1
      q(i,4) =  0; 
      tr = trace(dcm(:,:,i));
      if (tr > 0)
          sqtrp1 = sqrt( tr + 1.0 );
          q(i,1) = 0.5*sqtrp1; 
          q(i,2) = (dcm(2, 3, i) - dcm(3, 2, i))/(2.0*sqtrp1);
          q(i,3) = (dcm(3, 1, i) - dcm(1, 3, i))/(2.0*sqtrp1); 
          q(i,4) = (dcm(1, 2, i) - dcm(2, 1, i))/(2.0*sqtrp1); 
      else
          d = diag(dcm(:,:,i));
          if ((d(2) >= d(1)) && (d(2) >= d(3)))
              % max value at dcm(2,2,i)
              if (dcm(3, 1, i) - dcm(1, 3, i) >= 0)
                  sqdip1 = sqrt(d(2) - d(1) - d(3) + 1.0 );
              else
                  sqdip1 = -sqrt(d(2) - d(1) - d(3) + 1.0 );
              end
              q(i,3) = 0.5*sqdip1; 
              if ( sqdip1 ~= 0 )
                  sqdip1 = 0.5/sqdip1;
              end
              q(i,1) = (dcm(3, 1, i) - dcm(1, 3, i))*sqdip1; 
              q(i,2) = (dcm(1, 2, i) + dcm(2, 1, i))*sqdip1; 
              q(i,4) = (dcm(2, 3, i) + dcm(3, 2, i))*sqdip1; 
          elseif (d(3) >= d(1))
              % max value at dcm(3,3,i)
              if (dcm(1, 2, i) - dcm(2, 1, i) >= 0)
                  sqdip1 = sqrt(d(3) - d(1) - d(2) + 1.0 );
              else
                  sqdip1 = -sqrt(d(3) - d(1) - d(2) + 1.0 );
              end
              q(i,4) = 0.5*sqdip1; 
              if ( sqdip1 ~= 0 )
                  sqdip1 = 0.5/sqdip1;
              end
              q(i,1) = (dcm(1, 2, i) - dcm(2, 1, i))*sqdip1;
              q(i,2) = (dcm(3, 1, i) + dcm(1, 3, i))*sqdip1; 
              q(i,3) = (dcm(2, 3, i) + dcm(3, 2, i))*sqdip1; 
          else
              % max value at dcm(1,1,i)
              if (dcm(2, 3, i) - dcm(3, 2, i) >= 0)
                  sqdip1 = sqrt(d(1) - d(2) - d(3) + 1.0 );
              else
                  sqdip1 = -sqrt(d(1) - d(2) - d(3) + 1.0 );
              end
              q(i,2) = 0.5*sqdip1;
              if ( sqdip1 ~= 0 )
                  sqdip1 = 0.5/sqdip1;
              end
              q(i,1) = (dcm(2, 3, i) - dcm(3, 2, i))*sqdip1; 
              q(i,3) = (dcm(1, 2, i) + dcm(2, 1, i))*sqdip1; 
              q(i,4) = (dcm(3, 1, i) + dcm(1, 3, i))*sqdip1; 
          end
      end
  end