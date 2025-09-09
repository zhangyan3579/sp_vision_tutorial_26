#include <iostream>
#include <string>
using namespace std;

class Material {
public:
    Material();                    
    virtual ~Material();      
    virtual void print();      
};

class Book : public Material {
private:
    std::string title;
public:
    Book(const std::string& t); 
    ~Book();                    
    void print() override;      
};

class Magazine : public Material {
private:
    int issueNumber;
public:
    Magazine(int issue);        
    ~Magazine();                 
    void print() override;      
};

int main() {
    const int numMaterials = 3;
    Material* library[numMaterials];

    cout << "--- 构造函数 ---" << endl;
    library[0] = new Material();
    library[1] = new Book("Brave New World");
    library[2] = new Magazine(5);

    cout << "\n--- 多态调用print ---" << endl;
    for (int i = 0; i < numMaterials; ++i) {
        library[i]->print(); // 多态调用
    }

    cout << "\n--- 析构函数，释放内存 ---" << endl;
    for (int i = 0; i < numMaterials; ++i) {
        delete library[i]; // 虚析构函数确保正确释放
    }

    cout << "\n--- 结束 ---" << endl;
    return 0;
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

Book::Book(const std::string& t) : title(t) {
    cout << "Book Constructor!" << endl;
}

Book::~Book() {
    cout << "Book Destructor!" << endl;
}

void Book::print() {
    cout << "This is a Book titled: " << title << endl;
}

Magazine::Magazine(int issue) : issueNumber(issue) {
    cout << "Magazine Constructor!" << endl;
}

Magazine::~Magazine() {
    cout << "Magazine Destructor!" << endl;
}

void Magazine::print() {
    cout << "This is a Magazine, Issue No.: " << issueNumber << endl;
}