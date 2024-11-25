public class KruskalTest {
    /**
 * Created by Yunhao on 15/11/2024.
 */
    public static void main(String[] args) {
        // 创建Kruskal实例，假设图有4个节点
        Kruskal kruskal = new Kruskal(4);
        System.out.println("Subregion1:");
        // 初始化边 (from, to, cost)，添加一些边以构建一个图
        kruskal.AddToAllEdges(0, 1, 4);
        kruskal.AddToAllEdges(0, 2, 4);
        kruskal.AddToAllEdges(0, 3, 6);
        kruskal.AddToAllEdges(1, 2, 5);
        kruskal.AddToAllEdges(1, 3, 2);
        kruskal.AddToAllEdges(2, 3, 2);

        // 执行Kruskal算法生成最小生成树 (MST)
        kruskal.performKruskal();
        
        // 打印生成的最小生成树的边
        kruskal.printFinalEdges();

        System.out.println("Subregion2:");
        Kruskal kruskal2 = new Kruskal(5);
        kruskal2.AddToAllEdges(0, 1, 3);
        kruskal2.AddToAllEdges(0, 2, 2);
        kruskal2.AddToAllEdges(1, 2, 3);
        kruskal2.AddToAllEdges(1, 3, 4);
        kruskal2.AddToAllEdges(2, 3, 4);
        kruskal2.AddToAllEdges(2, 4, 15);
        kruskal2.AddToAllEdges(3, 4, 8);

        // 执行Kruskal算法生成最小生成树 (MST)
        kruskal2.performKruskal();
        
        // 打印生成的最小生成树的边
        kruskal2.printFinalEdges();

        System.out.println("Subregion3:");
        Kruskal kruskal3 = new Kruskal(5);
        kruskal3.AddToAllEdges(0, 1, 4);
        kruskal3.AddToAllEdges(0, 2, 5);
        kruskal3.AddToAllEdges(0, 3, 15);
        kruskal3.AddToAllEdges(0, 4, 16);
        kruskal3.AddToAllEdges(1, 2, 4);
        kruskal3.AddToAllEdges(1, 3, 10);
        kruskal3.AddToAllEdges(1, 4, 12);
        kruskal3.AddToAllEdges(2, 3, 12);
        kruskal3.AddToAllEdges(2, 4, 10);
        kruskal3.AddToAllEdges(3, 4, 4);

        // 执行Kruskal算法生成最小生成树 (MST)
        kruskal3.performKruskal();
        
        // 打印生成的最小生成树的边
        kruskal3.printFinalEdges();

        System.out.println("Subregion4:");
        Kruskal kruskal4 = new Kruskal(3);
        kruskal4.AddToAllEdges(0, 1, 6);
        kruskal4.AddToAllEdges(0, 2, 8);
        kruskal4.AddToAllEdges(1, 2, 10);


        // 执行Kruskal算法生成最小生成树 (MST)
        kruskal4.performKruskal();
        
        // 打印生成的最小生成树的边
        kruskal4.printFinalEdges();
    }
}
