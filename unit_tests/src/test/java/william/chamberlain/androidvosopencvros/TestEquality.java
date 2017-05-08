package william.chamberlain.androidvosopencvros;

public class TestEquality {

    public static void main(String[] args) {
        TestOnThis a = new TestOnThis("alice",114,114);
        TestOnThis b = new TestOnThis("bob",115,115);
        TestOnThis c = b;
        c.aNumber = 1000;
        c.anotherNumber = 2000;
        System.out.println(a);
        System.out.println(b);
        System.out.println(c);  //  c  is now the same as b (has same state), as expected
    }


}

class TestOnThis{
    String name;
    int aNumber;
    Integer anotherNumber;

    TestOnThis(String name, int aNumber, Integer anotherNumber) {
        this.name = name;
        this.aNumber = aNumber;
        this.anotherNumber = anotherNumber;
    }

    @Override
    public String toString() {
        return "TestOnThis{" +
                "name='" + name + '\'' +
                ", aNumber=" + aNumber +
                ", anotherNumber=" + anotherNumber +
                '}';
    }
}