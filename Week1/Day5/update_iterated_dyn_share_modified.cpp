// h_dyn_share -> boxminus -> SO3/S2修正 -> dx_ -> boxplus -> 收敛判断
void update_iterated_dyn_share_modified(double R, double &solve_time) {
		dyn_share_datastruct<scalar_type> dyn_share;
		dyn_share.valid = true;
		dyn_share.converge = true;
		int t = 0;
		// 保存 IMU propagation 后的预测状态。
		// 后面 boxminus 会拿当前迭代状态 x_ 和这个预测状态做差。
		state x_propagated = x_;

		cov P_propagated = P_;
		int dof_Measurement; 

		Matrix<scalar_type, n, 1> K_h;
		Matrix<scalar_type, n, n> K_x; 
		
		vectorized_state dx_new = vectorized_state::Zero();
		// IEKF 迭代循环。每一轮都会重新算残差、算增量、boxplus 更新状态。
		for(int i=-1; i<maximum_iter; i++)
		{
			dyn_share.valid = true;	
			// 用当前状态 x_ 计算 LiDAR 点面残差 h 和雅可比 h_x。
			// 你的动态点权重以后主要会影响这里产生的残差/雅可比。
			//  h_share_model提供观测信息。
			h_dyn_share(x_, dyn_share);

			if(! dyn_share.valid)
			{
				continue; 
			}

			//Matrix<scalar_type, Eigen::Dynamic, 1> h = h_dyn_share(x_, dyn_share);
			#ifdef USE_sparse
				spMt h_x_ = dyn_share.h_x.sparseView();
			#else
				Eigen::Matrix<scalar_type, Eigen::Dynamic, 12> h_x_ = dyn_share.h_x;
			#endif
			double solve_start = omp_get_wtime();
			dof_Measurement = h_x_.rows();
			vectorized_state dx;
			// boxminus：把“当前状态 x_”和“预测状态 x_propagated”的差变成向量 dx。
			// 普通向量状态可以直接相减；SO(3) 旋转不能直接 R-R，要用流形上的差，比如 Log(R_pred^{-1}R)。
			x_.boxminus(dx, x_propagated);
			// dx_new 先等于 dx，后面会对 SO(3)/S2 这些流形状态对应的块做修正。
			dx_new = dx;
			
			
			
			
			P_ = P_propagated;
			
			// 必下面这段专门处理 SO(3) 状态。
			// 状态里凡是 SO(3)，它的 dx 块不能像普通向量一样直接用。
			Matrix<scalar_type, 3, 3> res_temp_SO3;
			MTK::vect<3, scalar_type> seg_SO3;
			// 遍历状态里的 SO(3) 变量，例如 IMU姿态 rot、外参旋转 offset_R_L_I。
			for (std::vector<std::pair<int, int> >::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++) {
				// idx 是这个 SO(3) 状态在大状态向量 dx 里的起始位置。
				int idx = (*it).first;
				int dim = (*it).second;
				for(int i = 0; i < 3; i++){
					// 从 dx 中取出这个 SO(3) 对应的 3维小扰动。
					seg_SO3(i) = dx(idx+i);
				}

				// 旋转状态的误差块要先修正。
				res_temp_SO3 = MTK::A_matrix(seg_SO3).transpose();
				// 真正修正 dx_new 中这个 SO(3) 的 3维块。
				// 这说明：SO(3) 状态不是普通向量，不能完全按普通加减法处理。
				dx_new.template block<3, 1>(idx, 0) = res_temp_SO3 * dx_new.template block<3, 1>(idx, 0);
				for(int i = 0; i < n; i++){
					P_. template block<3, 1>(idx, i) = res_temp_SO3 * (P_. template block<3, 1>(idx, i));	
				}
				for(int i = 0; i < n; i++){
					P_. template block<1, 3>(i, idx) =(P_. template block<1, 3>(i, idx)) *  res_temp_SO3.transpose();	
				}

			// S2也不是普通向量
			Matrix<scalar_type, 2, 2> res_temp_S2;
			MTK::vect<2, scalar_type> seg_S2;
			for (std::vector<std::pair<int, int> >::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++) {
				// idx 是这个 SO(3) 状态在大状态向量 dx 里的起始位置。
				int idx = (*it).first;
				int dim = (*it).second;
				for(int i = 0; i < 2; i++){
					seg_S2(i) = dx(idx + i);
				}

				Eigen::Matrix<scalar_type, 2, 3> Nx;
				Eigen::Matrix<scalar_type, 3, 2> Mx;
				x_.S2_Nx_yy(Nx, idx);
				x_propagated.S2_Mx(Mx, seg_S2, idx);
				res_temp_S2 = Nx * Mx; 
				dx_new.template block<2, 1>(idx, 0) = res_temp_S2 * dx_new.template block<2, 1>(idx, 0);
				for(int i = 0; i < n; i++){
					P_. template block<2, 1>(idx, i) = res_temp_S2 * (P_. template block<2, 1>(idx, i));	
				}
				for(int i = 0; i < n; i++){
					P_. template block<1, 2>(i, idx) = (P_. template block<1, 2>(i, idx)) * res_temp_S2.transpose();
				}
			}
			//Matrix<scalar_type, n, Eigen::Dynamic> K_;
			//Matrix<scalar_type, n, 1> K_h;
			//Matrix<scalar_type, n, n> K_x; 

			/*
			//无论走哪个分支，最后都会得到 K_h 和 K_x。
			if(n > dof_Measurement)
			{
				K_= P_ * h_x_.transpose() * (h_x_ * P_ * h_x_.transpose()/R + Eigen::Matrix<double, Dynamic, Dynamic>::Identity(dof_Measurement, dof_Measurement)).inverse()/R;
			}
			else
			{
				K_= (h_x_.transpose() * h_x_ + (P_/R).inverse()).inverse()*h_x_.transpose();
			}
			*/

			if(n > dof_Measurement)
			{
			//#ifdef USE_sparse
				//Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> K_temp = h_x * P_ * h_x.transpose();
				//spMt R_temp = h_v * R_ * h_v.transpose();
				//K_temp += R_temp;
				Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> h_x_cur = Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic>::Zero(dof_Measurement, n);
				h_x_cur.topLeftCorner(dof_Measurement, 12) = h_x_;
				/*
				h_x_cur.col(0) = h_x_.col(0);
				h_x_cur.col(1) = h_x_.col(1);
				h_x_cur.col(2) = h_x_.col(2);
				h_x_cur.col(3) = h_x_.col(3);
				h_x_cur.col(4) = h_x_.col(4);
				h_x_cur.col(5) = h_x_.col(5);
				h_x_cur.col(6) = h_x_.col(6);
				h_x_cur.col(7) = h_x_.col(7);
				h_x_cur.col(8) = h_x_.col(8);
				h_x_cur.col(9) = h_x_.col(9);
				h_x_cur.col(10) = h_x_.col(10);
				h_x_cur.col(11) = h_x_.col(11);
				*/
				Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> K_ = P_ * h_x_cur.transpose() * (h_x_cur * P_ * h_x_cur.transpose()/R + Eigen::Matrix<double, Dynamic, Dynamic>::Identity(dof_Measurement, dof_Measurement)).inverse()/R;
				K_h = K_ * dyn_share.h;
				K_x = K_ * h_x_cur;
			//#else
			//	K_= P_ * h_x.transpose() * (h_x * P_ * h_x.transpose() + h_v * R * h_v.transpose()).inverse();
			//#endif
			}
			else
			{
			#ifdef USE_sparse
				//Eigen::Matrix<scalar_type, n, n> b = Eigen::Matrix<scalar_type, n, n>::Identity();
				//Eigen::SparseQR<Eigen::SparseMatrix<scalar_type>, Eigen::COLAMDOrdering<int>> solver; 
				spMt A = h_x_.transpose() * h_x_;
				cov P_temp = (P_/R).inverse();
				P_temp. template block<12, 12>(0, 0) += A;
				P_temp = P_temp.inverse();
				/*
				Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> h_x_cur = Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic>::Zero(dof_Measurement, n);
				h_x_cur.col(0) = h_x_.col(0);
				h_x_cur.col(1) = h_x_.col(1);
				h_x_cur.col(2) = h_x_.col(2);
				h_x_cur.col(3) = h_x_.col(3);
				h_x_cur.col(4) = h_x_.col(4);
				h_x_cur.col(5) = h_x_.col(5);
				h_x_cur.col(6) = h_x_.col(6);
				h_x_cur.col(7) = h_x_.col(7);
				h_x_cur.col(8) = h_x_.col(8);
				h_x_cur.col(9) = h_x_.col(9);
				h_x_cur.col(10) = h_x_.col(10);
				h_x_cur.col(11) = h_x_.col(11);
				*/
				K_ = P_temp. template block<n, 12>(0, 0) * h_x_.transpose();
				K_x = cov::Zero();
				K_x. template block<n, 12>(0, 0) = P_inv. template block<n, 12>(0, 0) * HTH;
				/*
				solver.compute(R_);
				Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> R_in_temp = solver.solve(b);
				spMt R_in =R_in_temp.sparseView();
				spMt K_temp = h_x.transpose() * R_in * h_x;
				cov P_temp = P_.inverse();
				P_temp += K_temp;
				K_ = P_temp.inverse() * h_x.transpose() * R_in;
				*/
			#else
				cov P_temp = (P_/R).inverse();
				//Eigen::Matrix<scalar_type, 12, Eigen::Dynamic> h_T = h_x_.transpose();
				Eigen::Matrix<scalar_type, 12, 12> HTH = h_x_.transpose() * h_x_; 
				P_temp. template block<12, 12>(0, 0) += HTH;
				/*
				Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> h_x_cur = Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic>::Zero(dof_Measurement, n);
				//std::cout << "line 1767" << std::endl;
				h_x_cur.col(0) = h_x_.col(0);
				h_x_cur.col(1) = h_x_.col(1);
				h_x_cur.col(2) = h_x_.col(2);
				h_x_cur.col(3) = h_x_.col(3);
				h_x_cur.col(4) = h_x_.col(4);
				h_x_cur.col(5) = h_x_.col(5);
				h_x_cur.col(6) = h_x_.col(6);
				h_x_cur.col(7) = h_x_.col(7);
				h_x_cur.col(8) = h_x_.col(8);
				h_x_cur.col(9) = h_x_.col(9);
				h_x_cur.col(10) = h_x_.col(10);
				h_x_cur.col(11) = h_x_.col(11);
				*/
				cov P_inv = P_temp.inverse();
				//std::cout << "line 1781" << std::endl;
				K_h = P_inv. template block<n, 12>(0, 0) * h_x_.transpose() * dyn_share.h;
				//std::cout << "line 1780" << std::endl;
				//cov HTH_cur = cov::Zero();
				//HTH_cur. template block<12, 12>(0, 0) = HTH;
				K_x.setZero(); // = cov::Zero();
				K_x. template block<n, 12>(0, 0) = P_inv. template block<n, 12>(0, 0) * HTH;
				//K_= (h_x_.transpose() * h_x_ + (P_/R).inverse()).inverse()*h_x_.transpose();
			#endif 
			}

			//K_x = K_ * h_x_;
			// dx_ 是本轮 IEKF 真正要加到状态上的增量。
			// 你可以把它理解成：残差 h、雅可比 H、协方差 P、旧误差 dx_new 综合后得到的状态修正量。
			Matrix<scalar_type, n, 1> dx_ = K_h + (K_x - Matrix<scalar_type, n, n>::Identity()) * dx_new; 
			state x_before = x_;
			// boxplus：把 dx_ 真正更新到状态 x_。
			// 普通向量：pos += dp, vel += dv, bias += db。
			// SO(3)：rot = rot * Exp(dtheta)，外参旋转同理。
			x_.boxplus(dx_);
			dyn_share.converge = true;
			for(int i = 0; i < n ; i++)
			{
				if(std::fabs(dx_[i]) > limit[i])
				{
					dyn_share.converge = false;
					break;
				}
			}
			if(dyn_share.converge) t++;
			
			if(!t && i == maximum_iter - 2)
			{
				dyn_share.converge = true;
			}
			if(t > 1 || i == maximum_iter - 1)
			{
				L_ = P_;
				//std::cout << "iteration time" << t << "," << i << std::endl; 
			// 下面这段专门处理 SO(3) 状态。
			// 你只要读懂：状态里凡是 SO(3)，它的 dx 块不能像普通向量一样直接用。
				Matrix<scalar_type, 3, 3> res_temp_SO3;
				MTK::vect<3, scalar_type> seg_SO3;
				for(typename std::vector<std::pair<int, int> >::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++) {
				// idx 是这个 SO(3) 状态在大状态向量 dx 里的起始位置。
					int idx = (*it).first;
					for(int i = 0; i < 3; i++){
						seg_SO3(i) = dx_(i + idx);
					}
				// 旋转状态的误差块要先修正。
					res_temp_SO3 = MTK::A_matrix(seg_SO3).transpose();
					for(int i = 0; i < n; i++){
						L_. template block<3, 1>(idx, i) = res_temp_SO3 * (P_. template block<3, 1>(idx, i)); 
					}
					// if(n > dof_Measurement)
					// {
					// 	for(int i = 0; i < dof_Measurement; i++){
					// 		K_.template block<3, 1>(idx, i) = res_temp_SO3 * (K_. template block<3, 1>(idx, i));
					// 	}
					// }
					// else
					// {
						for(int i = 0; i < 12; i++){
							K_x. template block<3, 1>(idx, i) = res_temp_SO3 * (K_x. template block<3, 1>(idx, i));
						}
					//}
					for(int i = 0; i < n; i++){
						L_. template block<1, 3>(i, idx) = (L_. template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
						P_. template block<1, 3>(i, idx) = (P_. template block<1, 3>(i, idx)) * res_temp_SO3.transpose();
					}
				}
				Matrix<scalar_type, 2, 2> res_temp_S2;
				MTK::vect<2, scalar_type> seg_S2;
				for(typename std::vector<std::pair<int, int> >::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++) {
				// idx 是这个 SO(3) 状态在大状态向量 dx 里的起始位置。
					int idx = (*it).first;

					for(int i = 0; i < 2; i++){
						seg_S2(i) = dx_(i + idx);
					}

					Eigen::Matrix<scalar_type, 2, 3> Nx;
					Eigen::Matrix<scalar_type, 3, 2> Mx;
					x_.S2_Nx_yy(Nx, idx);
					x_propagated.S2_Mx(Mx, seg_S2, idx);
					res_temp_S2 = Nx * Mx; 
					for(int i = 0; i < n; i++){
						L_. template block<2, 1>(idx, i) = res_temp_S2 * (P_. template block<2, 1>(idx, i)); 
					}
					// if(n > dof_Measurement)
					// {
					// 	for(int i = 0; i < dof_Measurement; i++){
					// 		K_. template block<2, 1>(idx, i) = res_temp_S2 * (K_. template block<2, 1>(idx, i));
					// 	}
					// }
					// else
					// {
						for(int i = 0; i < 12; i++){
							K_x. template block<2, 1>(idx, i) = res_temp_S2 * (K_x. template block<2, 1>(idx, i));
						}
					//}
					for(int i = 0; i < n; i++){
						L_. template block<1, 2>(i, idx) = (L_. template block<1, 2>(i, idx)) * res_temp_S2.transpose();
						P_. template block<1, 2>(i, idx) = (P_. template block<1, 2>(i, idx)) * res_temp_S2.transpose();
					}
				}

				// if(n > dof_Measurement)
				// {
				// 	Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> h_x_cur = Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic>::Zero(dof_Measurement, n);
				// 	h_x_cur.topLeftCorner(dof_Measurement, 12) = h_x_;
				// 	/*
				// 	h_x_cur.col(0) = h_x_.col(0);
				// 	h_x_cur.col(1) = h_x_.col(1);
				// 	h_x_cur.col(2) = h_x_.col(2);
				// 	h_x_cur.col(3) = h_x_.col(3);
				// 	h_x_cur.col(4) = h_x_.col(4);
				// 	h_x_cur.col(5) = h_x_.col(5);
				// 	h_x_cur.col(6) = h_x_.col(6);
				// 	h_x_cur.col(7) = h_x_.col(7);
				// 	h_x_cur.col(8) = h_x_.col(8);
				// 	h_x_cur.col(9) = h_x_.col(9);
				// 	h_x_cur.col(10) = h_x_.col(10);
				// 	h_x_cur.col(11) = h_x_.col(11);
				// 	*/
				// 	P_ = L_ - K_*h_x_cur * P_;
				// }
				// else
				//{
					P_ = L_ - K_x.template block<n, 12>(0, 0) * P_.template block<12, n>(0, 0);
				//}
				solve_time += omp_get_wtime() - solve_start;
				return;
			}
			solve_time += omp_get_wtime() - solve_start;
		}
	}
