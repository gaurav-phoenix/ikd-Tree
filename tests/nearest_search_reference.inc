// Original e2e3f4e9d3b95a9e66b1ba83dc98d4a05ed8a3c4 implementation,
// retained verbatim except the test-only member name.
template <typename PointType>
void KD_TREE<PointType>::Nearest_Search_Reference(PointType point, int k_nearest, PointVector &Nearest_Points, vector<float> &Point_Distance, float max_dist)
{
    MANUAL_HEAP q(2 * k_nearest);
    q.clear();
    vector<float>().swap(Point_Distance);
    if (Rebuild_Ptr == nullptr || *Rebuild_Ptr != Root_Node)
    {
        Search(Root_Node, k_nearest, point, q, max_dist);
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
        Search(Root_Node, k_nearest, point, q, max_dist);
        pthread_mutex_lock(&search_flag_mutex);
        search_mutex_counter -= 1;
        pthread_mutex_unlock(&search_flag_mutex);
    }
    int k_found = min(k_nearest, int(q.size()));
    PointVector().swap(Nearest_Points);
    vector<float>().swap(Point_Distance);
    for (int i = 0; i < k_found; i++)
    {
        Nearest_Points.insert(Nearest_Points.begin(), q.top().point);
        Point_Distance.insert(Point_Distance.begin(), q.top().dist);
        q.pop();
    }
    return;
}
