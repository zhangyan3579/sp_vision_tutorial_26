#include <iostream>
#include <string>
using namespace std;

class Material {
public:
    Material();
    ~Material();
    void print();
};

class Book : public Material {
private:
    string title;
public:
    Book(const string& t);
    ~Book();
    void print();
};

int main() {
    cout << "--- 构造函数 ---" << endl;
    Book my_book("Further Mathematics");

    cout << "\n--- print函数 ---" << endl;
    my_book.print();

    cout << "\n--- 自动进行析构函数 ---" << endl;
    return 0;
}

Book::Book(const string& t) : title(t) {
    cout << "Book Constructor!" << endl;
}

Book::~Book() {
    cout << "Book Destructor!" << endl;
}

void Book::print() {
    cout << "This is a Book object! It's name is " << title << endl;
}

Material::Material() {
    cout << "Material Default Constructor!" << endl;
}

Material::~Material() {
    cout << "Material Destructor!" << endl;
}

void Material::print() {
    cout << "This is a Material object!" << endl;
}

