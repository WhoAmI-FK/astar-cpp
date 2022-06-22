/*
*   This code is implemented studying the tutorial by AmbushedRacoon
*   Link for the tutorial: https://www.youtube.com/watch?v=7QQGVm6A1Iww
*   Author: Dzmitry
*   Student: WhoAmI-FK 
*/
#include <iostream>
#include <algorithm>
#include <fstream>
#include <string>
#include <vector>
#include <optional>
#include <stack>
#include <map>
#include <windows.h>
#include <wincodec.h>
#include <gdiplus.h>

const auto INF = (std::numeric_limits<int>::max)();
const auto INVALID_INDEX = (std::numeric_limits<std::size_t>::max)();


struct Edge
{
    int weight;
    std::size_t index_to;
};

struct Node 
{
    int id;
    int x;
    int y;
    std::vector<Edge> edges;
    std::size_t prev_index;
    int weight;
    bool visited;
};

struct Graph
{
    std::vector<Node> nodes;
    std::size_t visited_counter = 0;
    std::size_t start_index = INVALID_INDEX;
    std::size_t end_index = INVALID_INDEX;
    void clear()
    {
        nodes.clear();
    }
    void clear_edges()
    {
        for (auto& node : nodes)
        {
            node.edges.clear();
        }
    }
    void init_start_values()
    {
        for (auto& node : nodes)
        {
            node.weight = INF;
            node.visited = false;
            node.prev_index = INF;
        }
    }
};

bool read_nodes(std::istream& istream, std::size_t nodes_count, Graph& graph_out)
{
    graph_out.nodes.clear();
    for (std::size_t i = 0; i < nodes_count; i++)
    {
        // int
        decltype(Node::id) id;
        istream >> id;
        graph_out.nodes.push_back({ id });
    }
    return true;
}

bool read_edges(std::istream& istream, std::size_t edges_count, Graph& graph_out)
{
    graph_out.clear_edges();
    for (std::size_t i = 0; i < edges_count; i++)
    {
        int start_id, end_id;
        int weight;

        istream >> start_id >> end_id;
        istream >> weight;
        auto& nodes_ref = graph_out.nodes;

        auto start_iter = std::find_if(nodes_ref.begin(), nodes_ref.end(), [start_id](const auto& node) { return node.id == start_id; });
        auto end_iter = std::find_if(nodes_ref.begin(), nodes_ref.end(), [end_id](const auto& node) { return node.id == end_id; });

        if (start_iter == nodes_ref.end() || end_iter == nodes_ref.end())
        {
            graph_out.clear_edges();
            return false;
        }
        std::size_t index = (end_iter - nodes_ref.begin());
        (*start_iter).edges.push_back(Edge{ weight,index });
    }
    return true;
}

std::vector<std::size_t> convert_graph_to_path(Graph& graph)
{
    std::vector<std::size_t> result;
    std::stack<std::size_t> tmp_path;
    std::size_t current_node = graph.end_index;
    while (current_node != INF)
    {
        tmp_path.push(current_node);
        current_node = graph.nodes[current_node].prev_index;
    }

    while (!tmp_path.empty())
    {
        result.push_back(tmp_path.top());
        tmp_path.pop();
    }
    return result;
}

int euristic_weight(const Node& current, const Node& end)
{
    return current.weight + abs(current.x - end.x) + abs(current.y - end.y);
}


std::vector<std::size_t> find_path_AStar(Graph& graph)
{
    graph.visited_counter = 0;
    graph.init_start_values();
    std::multimap<int, std::size_t> min_weight_map;
    graph.nodes[graph.start_index].weight = 0;
    min_weight_map.insert({ 0, graph.start_index });
    while (!min_weight_map.empty())
    {
        auto [current_key, current_index] = *(min_weight_map.begin());
        int current_weight = graph.nodes[current_index].weight;
        min_weight_map.erase(min_weight_map.begin());
        if (graph.nodes[current_index].visited)
        {
            continue;
        }
        graph.nodes[current_index].visited = true;
        graph.visited_counter++;
        if (current_index == graph.end_index)
        {
            break;
        }
        for (std::size_t i = 0; i < graph.nodes[current_index].edges.size(); i++)
        {
            std::size_t index_to = graph.nodes[current_index].edges[i].index_to;
            int edged_weight = graph.nodes[current_index].edges[i].weight;
            if (!graph.nodes[index_to].visited)
            {
                if (current_weight + edged_weight < graph.nodes[index_to].weight)
                {
                    graph.nodes[index_to].weight = current_weight + edged_weight;
                    graph.nodes[index_to].prev_index = current_index;
                    min_weight_map.insert({ euristic_weight(graph.nodes[index_to],graph.nodes[graph.end_index]),index_to });
                }
            }
        }
    }
    return convert_graph_to_path(graph);
}

Graph read_from_image(const TCHAR* filename)
{
    Graph result;

    Gdiplus::GdiplusStartupInput gdiplusStartupInput;
    ULONG_PTR gdiplusToken;
    Gdiplus::GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);
    {
        Gdiplus::Bitmap input_bmp(filename);

        auto width = input_bmp.GetWidth();
        auto height = input_bmp.GetHeight();

        result.nodes.resize(width * height);
        for (INT i = 0; i < width; i++)
        {
            for (INT j = 0; j < height; j++)
            {
                auto node_index = i * height + j;
                result.nodes[node_index].x = i;
                result.nodes[node_index].y = j;
                Gdiplus::Color current_color;
                auto status = input_bmp.GetPixel(i, j, &current_color);
                if (current_color.GetValue() == 0xFF00FF00 && result.start_index == INVALID_INDEX)
                {
                    result.start_index = node_index;
                }
                if (current_color.GetValue() == Gdiplus::Color::Red && result.end_index == INVALID_INDEX)
                {
                    result.end_index = node_index;
                }
                if (current_color.GetValue() == Gdiplus::Color::Black)
                {
                    continue;
                }
                Gdiplus::Color neighbour_color;
                for (int vert_delta = -1; vert_delta <= 1; vert_delta++)
                {
                    for (int hor_delta = -1; hor_delta <= 1; hor_delta++)
                    {
                        if (abs(vert_delta) == abs(hor_delta))
                        {
                            continue;
                        }
                        if (input_bmp.GetPixel(i + vert_delta, j + hor_delta, &neighbour_color) == Gdiplus::Status::Ok)
                        {
                            if (neighbour_color.GetValue() != Gdiplus::Color::Black)
                            {
                                auto neighbour_index = (i + vert_delta) * height + (j + hor_delta);
                                result.nodes[node_index].edges.push_back({ 0, neighbour_index });
                            }
                        }
                    }

                }
            }
        }
    }
    Gdiplus::GdiplusShutdown(gdiplusToken);

    return result;
}

int GetEncoderClsid(const WCHAR* format, CLSID* pClsid)
{
    UINT  num = 0;          // number of image encoders
    UINT  size = 0;         // size of the image encoder array in bytes

    Gdiplus::ImageCodecInfo* pImageCodecInfo = NULL;

    Gdiplus::GetImageEncodersSize(&num, &size);
    if (size == 0)
        return -1;  // Failure

    pImageCodecInfo = (Gdiplus::ImageCodecInfo*)(malloc(size));
    if (pImageCodecInfo == NULL)
        return -1;  // Failure

    GetImageEncoders(num, size, pImageCodecInfo);

    for (UINT j = 0; j < num; ++j)
    {
        if (wcscmp(pImageCodecInfo[j].MimeType, format) == 0)
        {
            *pClsid = pImageCodecInfo[j].Clsid;
            free(pImageCodecInfo);
            return j;  // Success
        }
    }

    free(pImageCodecInfo);
    return -1;  // Failure
}

void draw_path_to_image(const Graph& graph, const std::vector<std::size_t>& path, const TCHAR* filename_source, const TCHAR* filename_out)
{
    Gdiplus::GdiplusStartupInput gdiplusStartupInput;
    ULONG_PTR gdiplusToken;
    Gdiplus::GdiplusStartup(&gdiplusToken, &gdiplusStartupInput, NULL);
    {
        Gdiplus::Bitmap input_bmp(filename_source);

        for (const auto& node : graph.nodes)
        {
            if (node.visited)
            {
                input_bmp.SetPixel(node.x, node.y, Gdiplus::Color::Blue);
            }
        }

        for (auto path_node_index : path)
        {
            input_bmp.SetPixel(graph.nodes[path_node_index].x, graph.nodes[path_node_index].y, Gdiplus::Color::Red);
        }

        CLSID pngClsid;
        GetEncoderClsid(TEXT("image/bmp"), &pngClsid);
        input_bmp.Save(filename_out, &pngClsid, NULL);

    }
    Gdiplus::GdiplusShutdown(gdiplusToken);
}


int main()
{
    auto image_graph = read_from_image(TEXT("input.bmp"));
    auto path = find_path_AStar(image_graph);
    draw_path_to_image(image_graph, path, TEXT("input.bmp"), TEXT("output.bmp"));
    std::cout << "Nodes visited: " << image_graph.visited_counter << std::endl;
    std::cout << std::endl;
}

