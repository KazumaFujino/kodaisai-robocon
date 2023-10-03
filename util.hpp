class UpEdge{
public:
  bool operator()(bool value){
    if(!oldValue && value){
      oldValue=value;
      return true;
    }else{
      oldValue=value;
      return false;
    }
  };
private:
  bool oldValue = true;
};
