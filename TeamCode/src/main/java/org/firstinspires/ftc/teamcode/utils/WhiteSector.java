public class WhiteSector implements Comparable<WhiteSector>{
    private int whiteCount;
    private int index;

    //default constructor
    public WhiteSector() {
        whiteCount = 0;
        index = 0;
    }

    public WhiteSector(int whiteCount, int index) {
        this.whiteCount = whiteCount;
        this.index = index;
    }

    public int getWhiteCount() {
        return whiteCount;
    }

    public void incrementWhiteCount() {
        whiteCount++;
    }

    public int getIndex() {
        return index;
    }

    static class WhiteComparator implements Comparator<WhiteSector> {
        @Override
        public int compare(WhiteSector a, WhiteSector b) {
            return -a.getWhiteCount() + b.getWhiteCount();
        }
    }
}
