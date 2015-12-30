#pragma once

#include <QMap>
#include <QList>
#include <QVector>
/*
 * TODO: add removing strategy, add insert/gc strategy
 */
template<typename KeyT, typename DataT>
class LimitedDataStorage
{
public:
  LimitedDataStorage(int MAX_NUM);
  virtual ~LimitedDataStorage();

  void insert(KeyT key, DataT data);
  void remove(KeyT key);
  void removeFirst();

  bool contains(KeyT key) const;
  DataT data(KeyT key) const;
  QList<KeyT> keys() const;
  int order(KeyT key) const;
  bool isEmpty() const;
  bool isFull() const;
  int size() const;

protected:
  int MAX_NUM_;
  QMap<KeyT, DataT> data_;
  QVector<KeyT> insertion_order_;
};

template<typename KeyT, typename DataT>
QList<KeyT> LimitedDataStorage<KeyT, DataT>::keys() const
{
  return data_.keys();
}

template<typename KeyT, typename DataT>
bool LimitedDataStorage<KeyT, DataT>::isFull() const
{
  return data_.size() >= MAX_NUM_;
}

template<typename KeyT, typename DataT>
int LimitedDataStorage<KeyT, DataT>::order(KeyT key) const
{
  return insertion_order_.indexOf(key);
}

template<typename KeyT, typename DataT>
LimitedDataStorage<KeyT, DataT>::LimitedDataStorage(int MAX_NUM)
  : MAX_NUM_(MAX_NUM)
{

}

template<typename KeyT, typename DataT>
LimitedDataStorage<KeyT, DataT>::~LimitedDataStorage()
{
  data_.clear();
  insertion_order_.clear();
}

template<typename KeyT, typename DataT>
int LimitedDataStorage<KeyT, DataT>::size() const
{
  return data_.size();
}

template<typename KeyT, typename DataT>
bool LimitedDataStorage<KeyT, DataT>::isEmpty() const
{
  return size() == 0;
}

template<typename KeyT, typename DataT>
bool LimitedDataStorage<KeyT, DataT>::contains(KeyT key) const
{
  return data_.contains(key);
}

template<typename KeyT, typename DataT>
void LimitedDataStorage<KeyT, DataT>::remove(KeyT key)
{
  data_.remove(key);
  insertion_order_.removeAll(key);
}

template<typename KeyT, typename DataT>
void LimitedDataStorage<KeyT, DataT>::removeFirst()
{
  if (insertion_order_.isEmpty()) return;
  KeyT key = insertion_order_.takeFirst();
  data_.remove(key);
}

template<typename KeyT, typename DataT>
void LimitedDataStorage<KeyT, DataT>::insert(KeyT key, DataT data)
{
  if (data_.size() == MAX_NUM_) removeFirst();
  data_.insert(key, data);
  insertion_order_.push_back(key);
}

template<typename KeyT, typename DataT>
DataT LimitedDataStorage<KeyT, DataT>::data(KeyT key) const
{
  return data_[key];
}
