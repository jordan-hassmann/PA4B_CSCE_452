


class World: 

    def __init__(self, world):
        self.resolution = world['resolution']

        self.map  = self.format_grid(world['map']) 

        cells = self.get_cells(self.map)
        edges = self.get_edges(cells)
        edges = self.merge_edges(edges)

        self.edges = self.adjust_resolution(edges, self.resolution)       

        print("World Generated!")

    
    
    def format_grid(self, grid):
        grid = [[ 100 if cell == '#' else 0 for cell in row.strip() ] for row in grid.strip().split('\n') ]
        return list(reversed(grid))
    

    def get_cells(self, grid): 

        cells = set()
        for y, row in enumerate(grid): 
            for x, val in enumerate(row): 
                if val == 100: 
                    cells.add((x, y))

        return cells
        

    def get_edges(self, cells: set): 

        edges = set()
        sides = [
            ((0, 0), (0, 1)),
            ((0, 1), (1, 1)), 
            ((1, 0), (1, 1)),
            ((0, 0), (1, 0)),
        ]
        for x, y in cells: 
            for i, cell in enumerate([
                (x-1, y), 
                (x, y+1), 
                (x+1, y), 
                (x, y-1), 
            ]): 
                
                if cell not in cells: 
                    (dx1, dy1), (dx2, dy2) = sides[i%4]
                    edges.add(((x+dx1, y+dy1), (x+dx2, y+dy2)))
        
        return edges 


    def merge_edges(self, edges: set): 

        def endpoint(v, d, p): 
            nv = not v
            s = None 
            i = 0
            while s is None or s in edges: 
                if s is not None: edges.remove(s)
                s = [0, 0]
                s[d!=1] = (p[0] + d*nv*(i+1), p[1] + d*v*(i+1))
                s[d==1] = (p[0] + d*nv*i, p[1] + d*v*i)
                s = tuple(s)
                i += 1

            return s[d!=1]


        segments = []
        while edges: 
            p1, p2 = edges.pop()
            segments.append([
                endpoint(p1[0] == p2[0],  1, p1), 
                endpoint(p1[0] == p2[0], -1, p2)
            ])

        return segments


    def adjust_resolution(self, edges, resolution): 

        segments = []
        for edge in edges: 
            (x1, y1), (x2, y2) = edge
            a = (x1 * resolution, y1 * resolution)
            b = (x2 * resolution, y2 * resolution) 
            segments.append([a, b])

        return segments

