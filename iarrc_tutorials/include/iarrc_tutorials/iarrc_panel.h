#pragma once


namespace iarrc_tutorials
{

    // FIXME TODO:
    // What should the panel demonstrate?
    // Show examples of each kind of button?
    // Use the group layout of the HuboMotionPanel?
    // Show the saving and loading functions?

    class IARRCPanel : public rviz:Panel
    {
        Q_OBJECT
    public:
        IARRCPanel(QWidget* parent=0);

        virtual void load(const rviz::COnfig& config);
        virtual void save(rviz::Config config) const;

    protected:
        
    protected Q_SLOTS:
        
    };

}

// Local Variables:
// mode: c++
// End:
