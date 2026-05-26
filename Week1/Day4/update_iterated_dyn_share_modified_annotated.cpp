	void update_iterated_dyn_share_modified(double R, double &solve_time) {
		// ESEKF 的迭代量测更新入口。
		// 输入：
		//   R          : LiDAR 观测噪声，代码里当作标量噪声使用。
		//   solve_time : 统计求解耗时，对滤波数学本身不重要。
		// 已有成员变量：
		//   x_ : IMU propagation 后的预测状态/当前迭代状态。
		//   P_ : IMU propagation 后的预测协方差/当前协方差。
		// 核心流程：
		//   1. h_dyn_share(x_, dyn_share) 计算 LiDAR 残差 h 和雅可比 H；
		//   2. 用 P、H、R 计算 Kalman gain；
		//   3. 得到误差状态 dx_；
		//   4. 用 x_.boxplus(dx_) 把误差加回状态。
		
		dyn_share_datastruct<scalar_type> dyn_share;
		// dyn_share 是“观测模型输出容器”：h_dyn_share 会把 LiDAR 残差 h、雅可比 H、valid/converge 等信息写进来。
		dyn_share.valid = true;      // 本轮观测默认有效，后面 h_dyn_share 可能会改成 false。
		dyn_share.converge = true;   // 本轮迭代默认收敛，后面根据 dx_ 是否超过阈值再判断。
		int t = 0;                  // 连续收敛计数。后面 t > 1 时认为迭代可以结束。
		state x_propagated = x_;    // 保存 IMU propagation 后的预测状态，作为本次 IEKF 迭代的基准状态。
		cov P_propagated = P_;      // 保存预测协方差，后面每轮迭代都从这个 P 重新修正。
		int dof_Measurement;        // 有效 LiDAR 残差数量，也就是 H 的行数。
		
		Matrix<scalar_type, n, 1> K_h;   // K_h = K * h。不是 K 本身，而是“卡尔曼增益乘残差”后的修正项。
		Matrix<scalar_type, n, n> K_x;   // K_x = K * H。也不是 K 本身，是后面 IEKF 更新公式里要用的 K H。
		
		vectorized_state dx_new = vectorized_state::Zero(); // 当前迭代状态 x_ 相对预测状态 x_propagated 的误差坐标。
		// IEKF 的迭代循环：每一轮都重新根据当前 x_ 计算残差和 H，再修正状态。
		for(int i=-1; i<maximum_iter; i++)
		{
			dyn_share.valid = true;      // 本轮观测默认有效，后面 h_dyn_share 可能会改成 false。	
			// 观测模型回调。
			// 输入当前状态 x_，输出 LiDAR 点面残差 dyn_share.h 和雅可比 dyn_share.h_x。
			// 这里相当于论文里把非线性观测模型 h(x) 在线性化，得到残差 r 和 H。
			h_dyn_share(x_, dyn_share);

			if(! dyn_share.valid)
			{
				// 如果这一轮 LiDAR 观测无效，就跳过本轮迭代，不进行 K 和状态更新。
				continue; 
			}

			// dyn_share.h_x 就是观测雅可比 H。注意：残差向量在 dyn_share.h 里。
			#ifdef USE_sparse
				spMt h_x_ = dyn_share.h_x.sparseView(); // 稀疏矩阵版本的 H。今天可以先不看 USE_sparse 分支。
			#else
				Eigen::Matrix<scalar_type, Eigen::Dynamic, 12> h_x_ = dyn_share.h_x; // h_x_ ≈ H，行数=残差数，12列=本观测直接涉及的误差状态维度。
			#endif
			double solve_start = omp_get_wtime();
			dof_Measurement = h_x_.rows(); // H 的行数，也就是这次有效残差/量测的数量。
			vectorized_state dx; // dx 表示当前迭代状态 x_ 相对预测状态 x_propagated 的误差。
			x_.boxminus(dx, x_propagated); // boxminus：在流形上做“相减”，得到误差状态 dx。
			dx_new = dx; // 保存当前线性化点相对预测点的误差，后面的 IEKF 修正公式要用。
			
			
			
			P_ = P_propagated; // 每轮迭代从预测协方差开始，再做流形坐标修正；不要理解成普通赋值回退。
			
			// 下面这一段是 SO(3) 姿态状态的误差坐标/协方差变换。
			Matrix<scalar_type, 3, 3> res_temp_SO3;
			MTK::vect<3, scalar_type> seg_SO3;
			for (std::vector<std::pair<int, int> >::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++) {
				int idx = (*it).first;
				int dim = (*it).second;
				for(int i = 0; i < 3; i++){
					seg_SO3(i) = dx(idx+i);
				}

				res_temp_SO3 = MTK::A_matrix(seg_SO3).transpose();
				dx_new.template block<3, 1>(idx, 0) = res_temp_SO3 * dx_new.template block<3, 1>(idx, 0);
				for(int i = 0; i < n; i++){
					P_. template block<3, 1>(idx, i) = res_temp_SO3 * (P_. template block<3, 1>(idx, i));	
				}
				for(int i = 0; i < n; i++){
					P_. template block<1, 3>(i, idx) =(P_. template block<1, 3>(i, idx)) *  res_temp_SO3.transpose();	
				}
			}

			// 下面这一段是 S2 状态的误差坐标/协方差变换。
			Matrix<scalar_type, 2, 2> res_temp_S2;
			MTK::vect<2, scalar_type> seg_S2;
			for (std::vector<std::pair<int, int> >::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++) {
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
			这段被注释掉的是原始/直观形式的 Kalman gain 写法：
			当状态维度 n 大于量测维度时，用 K = P H^T (H P H^T + R)^(-1)；
			否则用信息形式/等价形式，避免求太大的逆。
			今天只需要认出：两种写法数学上都是为了得到 K 或 K*h、K*H。

			// 情况 A：状态维度 n > 量测维度。
			// 这时显式构造完整 K_ 比较方便，形式接近标准 Kalman gain。
			if(n > dof_Measurement)
			{
				K_= P_ * h_x_.transpose() * (h_x_ * P_ * h_x_.transpose()/R + Eigen::Matrix<double, Dynamic, Dynamic>::Identity(dof_Measurement, dof_Measurement)).inverse()/R;
			}
			else
			{
				// 情况 B：量测维度更多，直接求 (H P H^T + R)^(-1) 代价较大。
				// 所以代码改用信息形式：先算 (P/R)^(-1) + H^T H，再求逆。
				// 这一支通常不会显式保存完整 K_，而是直接算 K_h = K*h 和 K_x = K*H。
				K_= (h_x_.transpose() * h_x_ + (P_/R).inverse()).inverse()*h_x_.transpose();
			}
			*/

			// 情况 A：状态维度 n > 量测维度。
			// 这时显式构造完整 K_ 比较方便，形式接近标准 Kalman gain。
			if(n > dof_Measurement)
			{
			//#ifdef USE_sparse
				//Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> K_temp = h_x * P_ * h_x.transpose();
				//spMt R_temp = h_v * R_ * h_v.transpose();
				//K_temp += R_temp;
				// h_x_ 只有 12 列，这里扩展成 dof_Measurement × n 的完整 H。
				// 直观理解：前 12 维有观测雅可比，其余状态维度对这个 LiDAR 观测的直接雅可比暂时置 0。
				Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> h_x_cur = Eigen::Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic>::Zero(dof_Measurement, n);
				h_x_cur.topLeftCorner(dof_Measurement, 12) = h_x_; // 把 H 的前 12 列填进去。
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
				
				// 这里是显式 Kalman gain K_。
				// 标准形式：K = P H^T (H P H^T + R I)^(-1)
				// 代码形式：P H^T * (H P H^T / R + I)^(-1) / R
				// 当 R 是标量时，这两种写法等价。
				Matrix<scalar_type, Eigen::Dynamic, Eigen::Dynamic> K_ = P_ * h_x_cur.transpose() * (h_x_cur * P_ * h_x_cur.transpose()/R + Eigen::Matrix<double, Dynamic, Dynamic>::Identity(dof_Measurement, dof_Measurement)).inverse()/R;
				K_h = K_ * dyn_share.h; // K_h = K * h。dyn_share.h 是 LiDAR 残差，不是 H。
				K_x = K_ * h_x_cur;     // K_x = K * H。后面 IEKF dx_ 公式会直接用 K_x。
			//#else
			//	K_= P_ * h_x.transpose() * (h_x * P_ * h_x.transpose() + h_v * R * h_v.transpose()).inverse();
			//#endif
			}
			else
			{
				// 情况 B：量测维度更多，直接求 (H P H^T + R)^(-1) 代价较大。
				// 所以代码改用信息形式：先算 (P/R)^(-1) + H^T H，再求逆。
				// 这一支通常不会显式保存完整 K_，而是直接算 K_h = K*h 和 K_x = K*H。
			#ifdef USE_sparse
				//Eigen::Matrix<scalar_type, n, n> b = Eigen::Matrix<scalar_type, n, n>::Identity();
				//Eigen::SparseQR<Eigen::SparseMatrix<scalar_type>, Eigen::COLAMDOrdering<int>> solver; 
				spMt A = h_x_.transpose() * h_x_;
				cov P_temp = (P_/R).inverse(); // 信息矩阵形式的先验项，可近似理解成 P^{-1} 相关项。
				P_temp. template block<12, 12>(0, 0) += A;
				P_temp = P_temp.inverse();
				/*
				// h_x_ 只有 12 列，这里扩展成 dof_Measurement × n 的完整 H。
				// 直观理解：前 12 维有观测雅可比，其余状态维度对这个 LiDAR 观测的直接雅可比暂时置 0。
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
				// 这里等价于计算 K_x = K * H，只是用 H^T H 的信息形式写出来。
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
				cov P_temp = (P_/R).inverse(); // 信息矩阵形式的先验项，可近似理解成 P^{-1} 相关项。
				//Eigen::Matrix<scalar_type, 12, Eigen::Dynamic> h_T = h_x_.transpose();
				Eigen::Matrix<scalar_type, 12, 12> HTH = h_x_.transpose() * h_x_; // H^T H，信息形式更新里的观测项。
				P_temp. template block<12, 12>(0, 0) += HTH; // 相当于把观测信息加到先验信息里。
				/*
				// h_x_ 只有 12 列，这里扩展成 dof_Measurement × n 的完整 H。
				// 直观理解：前 12 维有观测雅可比，其余状态维度对这个 LiDAR 观测的直接雅可比暂时置 0。
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
				cov P_inv = P_temp.inverse(); // 求回类似后验协方差/中间矩阵。
				//std::cout << "line 1781" << std::endl;
				// 这里没有显式写 K_，但这一整行等价于 K * h。
				K_h = P_inv. template block<n, 12>(0, 0) * h_x_.transpose() * dyn_share.h;
				//std::cout << "line 1780" << std::endl;
				//cov HTH_cur = cov::Zero();
				//HTH_cur. template block<12, 12>(0, 0) = HTH;
				K_x.setZero(); // 初始化 K_x = K * H。
				// 这里等价于计算 K_x = K * H，只是用 H^T H 的信息形式写出来。
				K_x. template block<n, 12>(0, 0) = P_inv. template block<n, 12>(0, 0) * HTH;
				//K_= (h_x_.transpose() * h_x_ + (P_/R).inverse()).inverse()*h_x_.transpose();
			#endif 
			}

			// 到这里为止，不管走哪个分支，都已经准备好了：
			//   K_h = K * h
			//   K_x = K * H
			// 所以下面可以统一写 IEKF 的误差状态更新。
			// IEKF 迭代更新量。
			// 普通 EKF 常见形式可以粗略记成 dx = K*h；
			// IEKF 因为在当前迭代点重新线性化，所以多了 (K_x - I) * dx_new 这一项。
			Matrix<scalar_type, n, 1> dx_ = K_h + (K_x - Matrix<scalar_type, n, n>::Identity()) * dx_new;
			state x_before = x_; // 保存更新前状态。这里后面没有明显使用，阅读时可忽略。
			x_.boxplus(dx_); // 把误差状态 dx_ 加回 x_。旋转等流形状态不是普通加法，而是 boxplus。
			dyn_share.converge = true; // 下面根据 dx_ 每一维是否足够小判断本轮是否收敛。   // 本轮迭代默认收敛，后面根据 dx_ 是否超过阈值再判断。
			// 如果 dx_ 任一维超过预设 limit，说明还没收敛，要继续迭代。
			for(int i = 0; i < n ; i++)
			{
				if(std::fabs(dx_[i]) > limit[i])
				{
					dyn_share.converge = false;
					break;
				}
			}
			if(dyn_share.converge) t++; // 连续收敛次数 +1。
			
			// 如果快到最大迭代次数还没有收敛，强行允许最后一次进入收尾更新，避免一直卡住。
			if(!t && i == maximum_iter - 2)
			{
				dyn_share.converge = true; // 快到最大迭代次数仍未连续收敛时，强制允许进入最后收尾。
			}

			// 满足以下任一条件就结束迭代：
			//   1. 连续收敛超过 1 次；
			//   2. 已经达到最大迭代次数。
			if(t > 1 || i == maximum_iter - 1)
			{
				L_ = P_; // 收尾阶段准备更新最终协方差。
				//std::cout << "iteration time" << t << "," << i << std::endl; 
				// 下面这两段是收尾阶段对 SO(3)/S2 流形状态相关矩阵的坐标修正。
				// 最终更新 P_ 前，还要把旋转/球面状态的扰动坐标对齐。
				Matrix<scalar_type, 3, 3> res_temp_SO3;
				MTK::vect<3, scalar_type> seg_SO3;
				for(typename std::vector<std::pair<int, int> >::iterator it = x_.SO3_state.begin(); it != x_.SO3_state.end(); it++) {
					int idx = (*it).first;
					for(int i = 0; i < 3; i++){
						seg_SO3(i) = dx_(i + idx);
					}
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

				// 下面这一段是 S2 状态的误差坐标/协方差变换。
			// 这里通常和重力方向这类单位球面状态有关。
			Matrix<scalar_type, 2, 2> res_temp_S2;
				MTK::vect<2, scalar_type> seg_S2;
				for(typename std::vector<std::pair<int, int> >::iterator it = x_.S2_state.begin(); it != x_.S2_state.end(); it++) {
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
					// 【收尾】协方差更新。可以粗略对应 EKF 里的 P = (I - K H) P。
				// 这里用 K_x = K H，并且只对前 12 维相关 block 做更新。
				P_ = L_ - K_x.template block<n, 12>(0, 0) * P_.template block<12, n>(0, 0);
				//}
				solve_time += omp_get_wtime() - solve_start; // 本轮未结束，累计耗时后继续下一轮迭代。 // 统计本次更新耗时。
				return; // 一次 iterated update 完成，退出函数。
			}
			solve_time += omp_get_wtime() - solve_start; // 本轮未结束，累计耗时后继续下一轮迭代。
		}
	}
