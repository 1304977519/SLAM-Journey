void KD_TREE<PointType>::Search(KD_TREE_NODE *root, int k_nearest, PointType point, MANUAL_HEAP &q, float max_dist)
{
    if (root == nullptr || root->tree_deleted)
        return;
    float cur_dist = calc_box_dist(root, point);
    float max_dist_sqr = max_dist * max_dist;
    if (cur_dist > max_dist_sqr)
        return;
    int retval;
    if (root->need_push_down_to_left || root->need_push_down_to_right)
    {
        retval = pthread_mutex_trylock(&(root->push_down_mutex_lock));
        if (retval == 0)
        {
            Push_Down(root);
            pthread_mutex_unlock(&(root->push_down_mutex_lock));
        }
        else
        {
            pthread_mutex_lock(&(root->push_down_mutex_lock));
            pthread_mutex_unlock(&(root->push_down_mutex_lock));
        }
    }
    if (!root->point_deleted)
    {
        float dist = calc_dist(point, root->point);
        if (dist <= max_dist_sqr && (q.size() < k_nearest || dist < q.top().dist))
        {
            if (q.size() >= k_nearest)
                q.pop();
            PointType_CMP current_point{root->point, dist};
            q.push(current_point);
        }
    }
    int cur_search_counter;
    float dist_left_node = calc_box_dist(root->left_son_ptr, point);
    float dist_right_node = calc_box_dist(root->right_son_ptr, point);
    if (q.size() < k_nearest || dist_left_node < q.top().dist && dist_right_node < q.top().dist)
    {
        if (dist_left_node <= dist_right_node)
        {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr)
            {
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);
            }
            else
            {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
            if (q.size() < k_nearest || dist_right_node < q.top().dist)
            {
                if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr)
                {
                    Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                }
                else
                {
                    pthread_mutex_lock(&search_flag_mutex);
                    while (search_mutex_counter == -1)
                    {
                        pthread_mutex_unlock(&search_flag_mutex);
                        usleep(1);
                        pthread_mutex_lock(&search_flag_mutex);
                    }
                    search_mutex_counter += 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                    Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                    pthread_mutex_lock(&search_flag_mutex);
                    search_mutex_counter -= 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                }
            }
        }
        else
        {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr)
            {
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
            }
            else
            {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
            if (q.size() < k_nearest || dist_left_node < q.top().dist)
            {
                if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr)
                {
                    Search(root->left_son_ptr, k_nearest, point, q, max_dist);
                }
                else
                {
                    pthread_mutex_lock(&search_flag_mutex);
                    while (search_mutex_counter == -1)
                    {
                        pthread_mutex_unlock(&search_flag_mutex);
                        usleep(1);
                        pthread_mutex_lock(&search_flag_mutex);
                    }
                    search_mutex_counter += 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                    Search(root->left_son_ptr, k_nearest, point, q, max_dist);
                    pthread_mutex_lock(&search_flag_mutex);
                    search_mutex_counter -= 1;
                    pthread_mutex_unlock(&search_flag_mutex);
                }
            }
        }
    }
    else
    {
        if (dist_left_node < q.top().dist)
        {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->left_son_ptr)
            {
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);
            }
            else
            {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);
                Search(root->left_son_ptr, k_nearest, point, q, max_dist);
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
        }
        if (dist_right_node < q.top().dist)
        {
            if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != root->right_son_ptr)
            {
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
            }
            else
            {
                pthread_mutex_lock(&search_flag_mutex);
                while (search_mutex_counter == -1)
                {
                    pthread_mutex_unlock(&search_flag_mutex);
                    usleep(1);
                    pthread_mutex_lock(&search_flag_mutex);
                }
                search_mutex_counter += 1;
                pthread_mutex_unlock(&search_flag_mutex);
                Search(root->right_son_ptr, k_nearest, point, q, max_dist);
                pthread_mutex_lock(&search_flag_mutex);
                search_mutex_counter -= 1;
                pthread_mutex_unlock(&search_flag_mutex);
            }
        }
    }
    return;
}