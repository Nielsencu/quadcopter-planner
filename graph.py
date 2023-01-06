
class Graph:
    def __init__(self):
        self.vertices = set()
        self.adjacentList = {}

    def getPath(self, src, dest):
        q = [[src]]
        while q:
            path = q.pop()
            vertex = path[-1]
            for newVertex in self.adjacentList[vertex]:
                newPath = path + [newVertex]
                if newVertex == dest:
                    return newPath
                q.append(newPath)
        return path

    def addVertex(self, vertex):
        self.vertices.add(vertex)
        self.adjacentList[vertex] = []

    def addLink(self, src, dest):
        self.adjacentList[src].append(dest)

