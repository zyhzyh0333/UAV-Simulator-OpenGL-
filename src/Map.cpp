#include "Map.h"

Map::Map(float map_s, float grid_s, float axis_s, std::string filename)
{
    map_scale = map_s;
    grid_scale = grid_s;
    axis_scale = axis_s;
    load_map_from_file(filename);
    init_blocks();
    // generate_path_points(50.0);
}

point_t Map::get_min_boundary() const
{
    return point_t(b_xmin, b_ymin, 0);
}
point_t Map::get_max_boundary() const
{
    return point_t(b_xmax, b_ymax, 0);
}

void Map::set_minmax(float b_xmin_n, float b_ymin_n, float b_xmax_n, float b_ymax_n) 
{
    b_xmin = b_xmin_n;
    b_ymin = b_ymin_n;
    b_xmax = b_xmax_n;
    b_ymax = b_ymax_n;
}


bool Map::is_inside_block(float testx, float testy) const
{
    for (auto &block : blocks) {
        float min_x = block.cen_x - block.side_len / 2.;
        float max_x = block.cen_x + block.side_len / 2.;
        float min_y = block.cen_z - block.side_len / 2.;
        float max_y = block.cen_z + block.side_len / 2.;

        if ((min_x <= testx && testx <= max_x) &&
            (min_y <= testy && testy <= max_y)) {
                return true;
        }
    }
    return false;
}

bool Map::is_inside_boundary(float testx, float testy) const
{
    int i, j;
    bool inside = false;
    int polysize = boundary_shape.size();
    for (i = 0, j = polysize - 1; i < polysize; j = i++)
    {
        double i_y = boundary_shape[i].y;
        double j_y = boundary_shape[j].y;
        double i_x = boundary_shape[i].x;
        double j_x = boundary_shape[j].x;
        if (((i_y > testy) != (j_y > testy)) &&
            (testx < (j_x - i_x) * (testy - i_y) / (j_y - i_y) + i_x))
        {
            inside = !inside;
        }
    }
    // cout<<"!!!!!!Isinside?: "<<inside<<endl;
    return inside;
}

void Map::load_map_from_file(const std::string &filename)
{

    // x, y center compensate
    float x_comp = 300.0;
    float y_comp = 300.0;
    float scale = 3.0;

    boundary_shape.clear();
    std::ifstream inFile;
    point_t tmp_node;
    float x_min = __FLT_MAX__;
    float x_max = -__FLT_MAX__;
    float y_min = __FLT_MAX__;
    float y_max = -__FLT_MAX__;

    inFile.open(filename);
    if (!inFile.is_open())
    {
        printf("ERROR: FILE NOT FOUND\n");
    }
    std::string line;

    float x, y;
    while (getline(inFile, line))
    {
        std::stringstream line_ss;
        line_ss.str(line);
        line_ss >> x >> y;
        // make some adjustment
        x = (x - x_comp) * scale;
        y = (y - y_comp) * scale;

        tmp_node.x = x;
        tmp_node.y = y;
        x_min = x > x_min ? x_min : x;
        y_min = y > y_min ? y_min : y;
        x_max = x > x_max ? x : x_max;
        y_max = y > y_max ? y : y_max;
        boundary_shape.push_back(tmp_node);
        //        copy_vec.push_back(tmp_node);
    }
    inFile.close();
    b_xmax = x_max;
    b_xmin = x_min;
    b_ymax = y_max;
    b_ymin = y_min;

    //    adjust_points(boundary_shape, 1.0);
}

void Map::set_boundary_shape(std::vector<point_t> new_shape) {
    boundary_shape = new_shape;
}

void Map::get_scale(float &gmap_scale, float &ggrid_scale, float &gaxis_scale) const
{
    gmap_scale = map_scale;
    ggrid_scale = grid_scale;
    gaxis_scale = axis_scale;
}

const std::vector<block_t> *Map::get_blocks() const
{
    return &blocks;
}

const std::vector<point_t> *Map::get_boundary() const
{
    return &boundary_shape;
}

void Map::add_block(block_t block)
{
    blocks.push_back(block);
}

void Map::init_blocks()
{
    blocks.clear();
    // add_block((block_t) {200, 0, 200, 300, 100});
    // add_block((block_t) {-100, 0, 200, 100, 100});
    // add_block((block_t) {100, 0, -100, 200, 100});
    // add_block((block_t) {-200, 0, -100, 200, 100});
    int n = 4;
    float k = std::min(b_xmax - b_xmin, b_ymax - b_ymin) / 10;
    for (int i = 0; i < n; i++)
    {
        int rand_num = rand();
        float cen_x = b_xmin + rand_num % (int)(b_xmax - b_xmin);
        float cen_y = b_ymin + rand_num % (int)(b_ymax - b_ymin);
        float side_len = k + (rand_num % 2) * k;
        float height = 50 + (rand_num % 3) * 50;
        add_block((block_t){cen_x, 0, cen_y, height, side_len});
    }
}

int get_ind(int x, int y, int side)
{
    return x % side + (y % side) * side;
}

void init_xy(std::vector<point_t> &coords, int side)
{
    for (size_t i = 0; i < coords.size(); i++)
    {
        coords.at(i).x = i % side;
        coords.at(i).y = i / side;
    }
}

void seed_corners(std::vector<point_t> &coords, int side)
{
    coords.at(get_ind(0, 0, side)).z = 0;
    coords.at(get_ind(side, 0, side)).z = 0;
    coords.at(get_ind(0, side, side)).z = 0;
    coords.at(get_ind(side, side, side)).z = 0;
}

void diamond_step(point_t p, int step, int s, float rand, std::vector<point_t> &terrain)
{
    int x = p.x;
    int y = p.y;
    int hstep = step / 2;
    float x1 = terrain.at(get_ind(x, y, s)).z;
    float x2 = terrain.at(get_ind(x + step, y, s)).z;
    float x3 = terrain.at(get_ind(x, y + step, s)).z;
    float x4 = terrain.at(get_ind(x + step, y + step, s)).z;

    float new_z = .25 * (x1 + x2 + x3 + x4) + rand;
    terrain.at(get_ind(x + hstep, y + hstep, s)).z = new_z;
}

void square_step(point_t p, int step, int s, float rand, std::vector<point_t> &terrain)
{
    int x = p.x;
    int y = p.y;
    int hstep = step / 2;
    float x1 = terrain.at(get_ind(x + hstep, y, s)).z;
    float x2 = terrain.at(get_ind(x, y + hstep, s)).z;
    float x3 = terrain.at(get_ind(x + step, y + hstep, s)).z;
    float x4 = terrain.at(get_ind(x + hstep, y + step, s)).z;

    float new_z = .25 * (x1 + x2 + x3 + x4) + rand;
    terrain.at(get_ind(x + hstep, y + hstep, s)).z += new_z;
}

void Map::init_terrain()
{
    init_xy(terrain, size);
    // https://en.wikipedia.org/wiki/Diamond-square_algorithm
    terrain.resize(pow(size, 2));
    seed_corners(terrain, size);

    int step = size;
    float random = (float)(rand() % 100);
    while (step > 1)
    {
        for (point_t &point : terrain)
        {
            diamond_step(point, step, size, random, terrain);
        }

        for (point_t &point : terrain)
        {
            square_step(point, step, size, random, terrain);
        }

        step /= 2;
    }
}